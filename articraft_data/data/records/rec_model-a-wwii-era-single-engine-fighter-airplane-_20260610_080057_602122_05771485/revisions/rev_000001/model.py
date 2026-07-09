from __future__ import annotations

# WWII-era single-engine fighter (Bearcat/Mustang style) in glossy medium blue
# with white "402" markings, yellow "B" on the fin, four-blade propeller.
#
# Frame conventions:
# - +X is the nose (flight) direction, +Y is the port (left) wing, +Z is up.
# - Fuselage centerline sits at z = CL_Z so that at the rest pose (blades in an
#   X configuration) the lowest blade tips graze the ground plane z ~= 0.
# - Articulations: continuous propeller spin about +X at the nose, raked
#   revolute rudder hinge on the fin trailing edge (+/-30 deg), and a one-piece
#   elevator on a lateral revolute hinge (+/-25 deg).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
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
CL_Z = 1.08  # fuselage centerline height above ground
NOSE_X = 3.40  # cowl front plane
TAIL_X = -4.20  # fuselage tail tip
SPAN_HALF = 5.25  # wing half-span (10.5 m total)
WING_ZC = -0.34  # wing mid-plane below fuselage centerline

PROP_TIP_R = 1.50  # propeller tip radius (3.0 m disc)
SPINNER_LEN = 0.78
SPINNER_BASE_X = 0.07  # local to the prop joint frame

# Rudder hinge: raked line x(z) = -3.78 - 0.18 z through (-3.78, 0, 0)
RUDDER_HINGE_X0 = -3.78
RUDDER_HINGE_SLOPE = 0.18
_rud_norm = math.sqrt(1.0 + RUDDER_HINGE_SLOPE**2)
RUDDER_AXIS = (-RUDDER_HINGE_SLOPE / _rud_norm, 0.0, 1.0 / _rud_norm)

ELEV_HINGE_X = -3.80
ELEV_ZC = 0.06

# Fuselage hull lathe control profile: (radius, z) with z measured from the
# tail tip (z=0) to the cowl front plane (z=7.6).
HULL_PROFILE = [
    (0.015, 0.00),
    (0.07, 0.15),
    (0.13, 0.70),
    (0.20, 1.40),
    (0.30, 2.40),
    (0.42, 3.60),
    (0.52, 4.80),
    (0.60, 5.90),
    (0.65, 6.60),
    (0.66, 7.00),
    (0.655, 7.35),
    (0.60, 7.60),
]


def _hull_radius_at_x(x: float) -> float:
    """Linear-interp hull radius at fuselage station x (centerline coords)."""
    z = x - TAIL_X
    pts = HULL_PROFILE
    if z <= pts[0][1]:
        return pts[0][0]
    for (r0, z0), (r1, z1) in zip(pts, pts[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return pts[-1][0]


# ---------------------------------------------------------------------------
# Airfoil section helpers
# ---------------------------------------------------------------------------
def _foil_pairs_full(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Cambered airfoil loop (rounded LE, near-sharp TE). 17 points."""
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


def _foil_pairs_blunt_back(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Symmetric fixed-surface loop ending blunt at the hinge plane. 13 points."""
    c = front - back
    upper = [
        (0.00, 0.00),
        (0.05, 0.45),
        (0.15, 0.75),
        (0.35, 1.00),
        (0.60, 0.92),
        (0.80, 0.75),
        (1.00, 0.55),
    ]
    lower = [
        (1.00, -0.55),
        (0.80, -0.75),
        (0.60, -0.92),
        (0.35, -1.00),
        (0.15, -0.75),
        (0.05, -0.45),
    ]
    return [(front - u * c, f * ht) for u, f in upper + lower]


def _foil_pairs_blunt_front(front: float, back: float, ht: float) -> list[tuple[float, float]]:
    """Movable control-surface loop: blunt at the hinge, sharp TE. 10 points."""
    c = front - back
    upper = [
        (0.00, 0.55),
        (0.20, 0.50),
        (0.45, 0.38),
        (0.70, 0.24),
        (1.00, 0.05),
    ]
    lower = [
        (1.00, -0.05),
        (0.70, -0.24),
        (0.45, -0.38),
        (0.20, -0.50),
        (0.00, -0.55),
    ]
    return [(front - u * c, f * ht) for u, f in upper + lower]


# ---------------------------------------------------------------------------
# Component meshes
# ---------------------------------------------------------------------------
def _hull_mesh() -> MeshGeometry:
    profile = sample_catmull_rom_spline_2d(HULL_PROFILE, samples_per_segment=8)
    profile = [(max(r, 0.0), z) for r, z in profile]
    profile.append((0.0, profile[-1][1]))  # close the cowl front face
    hull = LatheGeometry(profile, segments=64)
    hull.rotate_y(math.pi / 2.0)  # lathe +Z axis -> +X (nose forward)
    hull.translate(TAIL_X, 0.0, 0.0)
    return hull


def _canopy_mesh() -> MeshGeometry:
    canopy = SphereGeometry(1.0, width_segments=28, height_segments=18)
    canopy.scale(0.78, 0.30, 0.45)
    canopy.translate(0.90, 0.0, 0.45)
    return canopy


def _tail_fairing_mesh() -> MeshGeometry:
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
            loop(0.02, -3.55, -4.34, 0.055),
            loop(0.28, -3.62, -4.30, 0.038),
        ]
    )


def _wing_mesh() -> MeshGeometry:
    # (y, center_x, chord, half_thickness); rounded tips via shrinking sections.
    specs = [
        (-5.25, 0.62, 0.30, 0.018),
        (-5.08, 0.66, 0.78, 0.034),
        (-4.55, 0.70, 1.18, 0.052),
        (-3.40, 0.74, 1.50, 0.072),
        (-1.30, 0.78, 1.92, 0.100),
        (0.00, 0.78, 1.98, 0.105),
        (1.30, 0.78, 1.92, 0.100),
        (3.40, 0.74, 1.50, 0.072),
        (4.55, 0.70, 1.18, 0.052),
        (5.08, 0.66, 0.78, 0.034),
        (5.25, 0.62, 0.30, 0.018),
    ]
    sections = []
    for y, xc, c, ht in specs:
        pairs = _foil_pairs_full(xc + c / 2.0, xc - c / 2.0, ht)
        sections.append([(x, y, WING_ZC + t) for x, t in pairs])
    # Ruled loft: spline lofting overshoots the tip sections and inflates span.
    return section_loft(sections, ruled=True)


def _fin_le(z: float) -> float:
    return -2.85 - 0.62 * z


def _fin_fixed_back(z: float) -> float:
    return RUDDER_HINGE_X0 - RUDDER_HINGE_SLOPE * z + 0.012


def _fin_ht(z: float) -> float:
    # half thickness from root (z=0.12) to tip (z=1.55)
    return max(0.018, 0.075 - 0.040 * (z - 0.12))


def _fin_section(z: float) -> list[tuple[float, float, float]]:
    pairs = _foil_pairs_blunt_back(_fin_le(z), _fin_fixed_back(z), _fin_ht(z))
    return [(x, t, z) for x, t in pairs]


def _fin_mesh_blue() -> MeshGeometry:
    return section_loft([_fin_section(z) for z in (0.12, 0.55, 0.95, 1.30)])


def _fin_mesh_yellow_tip() -> MeshGeometry:
    return section_loft([_fin_section(z) for z in (1.30, 1.46, 1.55)])


def _rudder_front(z: float) -> float:
    return RUDDER_HINGE_X0 - RUDDER_HINGE_SLOPE * z - 0.012


def _rudder_te(z: float) -> float:
    return -4.33 + 0.05 * z


def _rudder_ht(z: float) -> float:
    return max(0.016, 0.050 - 0.028 * (z - 0.30))


def _rudder_section(z: float) -> list[tuple[float, float, float]]:
    pairs = _foil_pairs_blunt_front(_rudder_front(z), _rudder_te(z), _rudder_ht(z))
    # Rudder geometry is authored relative to the hinge anchor (-3.78, 0, 0).
    return [(x - RUDDER_HINGE_X0, t, z) for x, t in pairs]


def _rudder_mesh_blue() -> MeshGeometry:
    return section_loft([_rudder_section(z) for z in (0.30, 0.75, 1.10, 1.30)])


def _rudder_mesh_yellow_tip() -> MeshGeometry:
    return section_loft([_rudder_section(z) for z in (1.30, 1.44, 1.52)])


def _rudder_hinge_barrel() -> MeshGeometry:
    barrel = CylinderGeometry(0.014, 1.00, radial_segments=16)
    tilt = math.atan2(RUDDER_HINGE_SLOPE, 1.0)
    barrel.rotate_y(-tilt)
    # center the barrel on the hinge line at mid rudder height
    t_mid = 0.80
    barrel.translate(-RUDDER_HINGE_SLOPE / _rud_norm * t_mid, 0.0, t_mid / _rud_norm)
    return barrel


def _stab_le(y: float) -> float:
    return -3.05 - 0.28 * abs(y)


def _stab_ht(y: float) -> float:
    return max(0.018, 0.045 - 0.0135 * abs(y))


def _stab_mesh() -> MeshGeometry:
    back = ELEV_HINGE_X + 0.012
    sections = []
    for y in (-1.85, -1.78, -1.55, -0.90, 0.0, 0.90, 1.55, 1.78, 1.85):
        ht = _stab_ht(y) * (0.55 if abs(y) > 1.80 else 1.0)
        pairs = _foil_pairs_blunt_back(_stab_le(y), back, ht)
        sections.append([(x, y, ELEV_ZC + t) for x, t in pairs])
    return section_loft(sections)


def _elevator_mesh() -> MeshGeometry:
    front = ELEV_HINGE_X - 0.012

    def te(y: float) -> float:
        return -4.28 + 0.22 * abs(y)

    sections = []
    for y in (-1.75, -1.50, -0.90, 0.0, 0.90, 1.50, 1.75):
        ht = 0.040 - 0.0126 * abs(y)
        pairs = _foil_pairs_blunt_front(front, te(y), ht)
        # Elevator geometry is authored relative to hinge frame (-3.80, 0, 0.06).
        sections.append([(x - ELEV_HINGE_X, y, t) for x, t in pairs])
    return section_loft(sections)


def _elevator_torque_tube() -> MeshGeometry:
    tube = CylinderGeometry(0.034, 0.62, radial_segments=16)
    tube.rotate_x(math.pi / 2.0)  # long axis Z -> Y
    return tube


def _spinner_mesh() -> MeshGeometry:
    profile = [
        (0.000, 0.00),  # closed base disk so the shaft connects to the shell
        (0.330, 0.00),
        (0.335, 0.10),
        (0.315, 0.28),
        (0.260, 0.48),
        (0.160, 0.66),
        (0.050, 0.76),
        (0.000, SPINNER_LEN),
    ]
    spinner = LatheGeometry(profile, segments=48)
    spinner.rotate_y(math.pi / 2.0)
    spinner.translate(SPINNER_BASE_X, 0.0, 0.0)
    return spinner


def _prop_shaft_mesh() -> MeshGeometry:
    shaft = CylinderGeometry(0.05, 0.18, radial_segments=20)
    shaft.rotate_y(math.pi / 2.0)
    shaft.translate(0.04, 0.0, 0.0)  # spans x in [-0.05, 0.13]
    return shaft


# Blade sections: (radius, chord, half_thickness, twist_rad)
_BLADE_BLACK = [
    (0.16, 0.130, 0.075, 1.00),
    (0.30, 0.180, 0.055, 0.85),
    (0.55, 0.225, 0.042, 0.62),
    (0.85, 0.235, 0.033, 0.46),
    (1.12, 0.200, 0.027, 0.37),
    (1.30, 0.165, 0.023, 0.33),
]
_BLADE_YELLOW = [
    (1.30, 0.165, 0.023, 0.33),
    (1.42, 0.115, 0.018, 0.30),
    (1.50, 0.035, 0.008, 0.28),
]


def _blade_loft(specs: list[tuple[float, float, float, float]]) -> MeshGeometry:
    sections = []
    for r, c, ht, tw in specs:
        pairs = _foil_pairs_full(c / 2.0, -c / 2.0, ht)
        loop = []
        for a, t in pairs:
            # chord coordinate a (in the rotation plane), thickness t; twist tw
            # tilts the chord toward the prop axis (+X) about the radial axis.
            x = a * math.sin(tw) + t * math.cos(tw)
            y = a * math.cos(tw) - t * math.sin(tw)
            loop.append((x, y, r))
        sections.append(loop)
    return section_loft(sections)


def _blade_set(specs: list[tuple[float, float, float, float]]) -> MeshGeometry:
    base = _blade_loft(specs)
    base.translate(0.30, 0.0, 0.0)  # blade plane behind the spinner mid-body
    merged = MeshGeometry()
    for k in range(4):
        blade = base.copy()
        blade.rotate_x(math.radians(45.0 + 90.0 * k))  # X pattern at rest
        merged.merge(blade)
    return merged


# ---------------------------------------------------------------------------
# Marking meshes (blocky 7-segment style digits / letter)
# ---------------------------------------------------------------------------
_SEGMENTS_BY_DIGIT = {
    "4": ("F", "B", "G", "C"),
    "0": ("A", "F", "B", "E", "C", "D"),
    "2": ("A", "B", "G", "E", "D"),
}


def _digit_mesh(ch: str, h: float, t: float, mirror: bool = False) -> MeshGeometry:
    w = 0.62 * h
    s = 0.17 * h
    seg_boxes = {
        "A": (0.0, h / 2 - s / 2, w, s),
        "D": (0.0, -h / 2 + s / 2, w, s),
        "G": (0.0, 0.0, w, s),
        "F": (-w / 2 + s / 2, h / 4, s, h / 2),
        "B": (w / 2 - s / 2, h / 4, s, h / 2),
        "E": (-w / 2 + s / 2, -h / 4, s, h / 2),
        "C": (w / 2 - s / 2, -h / 4, s, h / 2),
    }
    mesh = MeshGeometry()
    for seg in _SEGMENTS_BY_DIGIT[ch]:
        cx, cy, dx, dy = seg_boxes[seg]
        if mirror:
            cx = -cx  # mirror the glyph so it reads correctly from the far side
        box = BoxGeometry((dx, dy, t))
        box.translate(cx, cy, 0.0)
        mesh.merge(box)
    return mesh


def _text_402_mesh(h: float, t: float, gap: float) -> MeshGeometry:
    w = 0.62 * h
    pitch = w + gap
    mesh = MeshGeometry()
    for i, ch in enumerate("402"):
        d = _digit_mesh(ch, h, t)
        d.translate((1 - i) * pitch, 0.0, 0.0)  # "4" forward (+x), "2" aft
        mesh.merge(d)
    return mesh


def _letter_b_mesh(h: float, t: float) -> MeshGeometry:
    w = 0.66 * h
    s = 0.17 * h
    boxes = [
        (-w / 2 + s / 2, 0.0, s, h),
        (0.0, h / 2 - s / 2, w, s),
        (0.0, 0.0, w, s),
        (0.0, -h / 2 + s / 2, w, s),
        (w / 2 - s / 2, h / 4, s, h / 2),
        (w / 2 - s / 2, -h / 4, s, h / 2),
    ]
    mesh = MeshGeometry()
    for cx, cy, dx, dy in boxes:
        box = BoxGeometry((dx, dy, t))
        box.translate(cx, cy, 0.0)
        mesh.merge(box)
    return mesh


def _fuselage_side_number(side: float) -> MeshGeometry:
    """White "402" plates wrapped onto one fuselage flank (side=+1 port).

    Each flank reads "402" nose-first to an outside observer: the port side
    needs mirrored glyphs (its reading direction is -X on screen), while the
    starboard side keeps plain glyphs but reverses the station order.
    """
    stations = (-0.34, -0.62, -0.90) if side > 0 else (-0.90, -0.62, -0.34)
    mesh = MeshGeometry()
    for x_k, ch in zip(stations, "402"):
        r_k = _hull_radius_at_x(x_k + 0.10)
        d = _digit_mesh(ch, 0.30, 0.09, mirror=side > 0)
        d.rotate_x(math.pi / 2.0)  # height -> Z, thickness -> Y
        d.translate(x_k, side * (r_k - 0.005), 0.05)
        mesh.merge(d)
    return mesh


def _wing_number(y_center: float, z_plate: float) -> MeshGeometry:
    text = _text_402_mesh(0.42, 0.05, 0.10)
    text.rotate_z(math.pi / 2.0)  # digits run spanwise
    text.translate(0.75, y_center, z_plate)
    return text


def _fin_letter(side: float) -> MeshGeometry:
    letter = _letter_b_mesh(0.26, 0.05)
    letter.rotate_x(math.pi / 2.0)
    letter.translate(-3.72, side * 0.045, 1.02)
    return letter


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wwii_fighter_airplane")

    blue = model.material("gloss_blue", rgba=(0.16, 0.34, 0.62, 1.0))
    white = model.material("marking_white", rgba=(0.93, 0.93, 0.93, 1.0))
    yellow = model.material("marking_yellow", rgba=(0.95, 0.78, 0.15, 1.0))
    black = model.material("blade_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("canopy_glass", rgba=(0.15, 0.22, 0.32, 1.0))
    steel = model.material("steel_gray", rgba=(0.38, 0.39, 0.42, 1.0))
    engine = model.material("engine_dark", rgba=(0.13, 0.13, 0.14, 1.0))

    cl = Origin(xyz=(0.0, 0.0, CL_Z))

    # --- fuselage (root) ---
    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_geometry(_hull_mesh(), "fuselage_hull"),
        origin=cl,
        material=blue,
        name="hull",
    )
    fuselage.visual(
        Cylinder(radius=0.52, length=0.06),
        origin=Origin(xyz=(3.42, 0.0, CL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=engine,
        name="engine_face",
    )
    fuselage.visual(
        mesh_from_geometry(_canopy_mesh(), "canopy_bubble"),
        origin=cl,
        material=glass,
        name="canopy",
    )
    fuselage.visual(
        mesh_from_geometry(_tail_fairing_mesh(), "tail_fairing"),
        origin=cl,
        material=blue,
        name="tail_fairing",
    )
    fuselage.visual(
        mesh_from_geometry(_fuselage_side_number(+1.0), "side_number_left"),
        origin=cl,
        material=white,
        name="side_number_left",
    )
    fuselage.visual(
        mesh_from_geometry(_fuselage_side_number(-1.0), "side_number_right"),
        origin=cl,
        material=white,
        name="side_number_right",
    )

    # --- wing ---
    wing = model.part("wing")
    wing.visual(
        mesh_from_geometry(_wing_mesh(), "wing_loft"),
        material=blue,
        name="wing_loft",
    )
    wing.visual(
        mesh_from_geometry(_wing_number(-2.80, -0.257), "wing_number_top"),
        material=white,
        name="wing_number_top",
    )
    wing.visual(
        mesh_from_geometry(_wing_number(2.80, -0.403), "wing_number_bottom"),
        material=white,
        name="wing_number_bottom",
    )
    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing,
        origin=cl,
    )

    # --- vertical fin ---
    tail_fin = model.part("tail_fin")
    tail_fin.visual(
        mesh_from_geometry(_fin_mesh_blue(), "fin_loft"),
        material=blue,
        name="fin_loft",
    )
    tail_fin.visual(
        mesh_from_geometry(_fin_mesh_yellow_tip(), "fin_tip"),
        material=yellow,
        name="fin_tip",
    )
    tail_fin.visual(
        mesh_from_geometry(_fin_letter(+1.0), "fin_letter_left"),
        material=yellow,
        name="fin_letter_left",
    )
    tail_fin.visual(
        mesh_from_geometry(_fin_letter(-1.0), "fin_letter_right"),
        material=yellow,
        name="fin_letter_right",
    )
    model.articulation(
        "fuselage_to_tail_fin",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail_fin,
        origin=cl,
    )

    # --- rudder (revolute, raked vertical hinge) ---
    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(_rudder_mesh_blue(), "rudder_loft"),
        material=blue,
        name="rudder_loft",
    )
    rudder.visual(
        mesh_from_geometry(_rudder_mesh_yellow_tip(), "rudder_tip"),
        material=yellow,
        name="rudder_tip",
    )
    rudder.visual(
        mesh_from_geometry(_rudder_hinge_barrel(), "rudder_hinge_barrel"),
        material=steel,
        name="hinge_barrel",
    )
    model.articulation(
        "fin_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tail_fin,
        child=rudder,
        origin=Origin(xyz=(RUDDER_HINGE_X0, 0.0, 0.0)),
        axis=RUDDER_AXIS,
        motion_limits=MotionLimits(
            effort=60.0, velocity=3.0, lower=-math.radians(30.0), upper=math.radians(30.0)
        ),
    )

    # --- horizontal stabilizer ---
    stabilizer = model.part("horizontal_stabilizer")
    stabilizer.visual(
        mesh_from_geometry(_stab_mesh(), "stabilizer_loft"),
        material=blue,
        name="stabilizer_loft",
    )
    model.articulation(
        "fuselage_to_stabilizer",
        ArticulationType.FIXED,
        parent=fuselage,
        child=stabilizer,
        origin=cl,
    )

    # --- elevator (one link across both stabilizer trailing edges) ---
    elevator = model.part("elevator")
    elevator.visual(
        mesh_from_geometry(_elevator_mesh(), "elevator_loft"),
        material=blue,
        name="elevator_loft",
    )
    elevator.visual(
        mesh_from_geometry(_elevator_torque_tube(), "elevator_torque_tube"),
        material=steel,
        name="torque_tube",
    )
    model.articulation(
        "stabilizer_to_elevator",
        ArticulationType.REVOLUTE,
        parent=stabilizer,
        child=elevator,
        origin=Origin(xyz=(ELEV_HINGE_X, 0.0, ELEV_ZC)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=3.0, lower=-math.radians(25.0), upper=math.radians(25.0)
        ),
    )

    # --- propeller assembly (spinner + 4 blades, continuous spin) ---
    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(_spinner_mesh(), "spinner_shell"),
        material=blue,
        name="spinner_shell",
    )
    propeller.visual(
        mesh_from_geometry(_prop_shaft_mesh(), "prop_shaft"),
        material=steel,
        name="prop_shaft",
    )
    propeller.visual(
        mesh_from_geometry(_blade_set(_BLADE_BLACK), "prop_blades"),
        material=black,
        name="blades",
    )
    propeller.visual(
        mesh_from_geometry(_blade_set(_BLADE_YELLOW), "prop_blade_tips"),
        material=yellow,
        name="blade_tips",
    )
    model.articulation(
        "nose_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(NOSE_X, 0.0, CL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=80.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    wing = object_model.get_part("wing")
    tail_fin = object_model.get_part("tail_fin")
    stabilizer = object_model.get_part("horizontal_stabilizer")
    elevator = object_model.get_part("elevator")
    rudder = object_model.get_part("rudder")
    propeller = object_model.get_part("propeller")

    prop_spin = object_model.get_articulation("nose_prop_spin")
    rudder_hinge = object_model.get_articulation("fin_to_rudder")
    elev_hinge = object_model.get_articulation("stabilizer_to_elevator")

    # --- intentional, scoped overlaps (real construction junctions) ---
    ctx.allow_overlap(
        wing,
        fuselage,
        elem_a="wing_loft",
        elem_b="hull",
        reason="One-piece low wing carries through the fuselage belly, as on the real aircraft.",
    )
    ctx.allow_overlap(
        tail_fin,
        fuselage,
        elem_a="fin_loft",
        elem_b="hull",
        reason="Fin root spar is buried in the tail cone.",
    )
    ctx.allow_overlap(
        tail_fin,
        fuselage,
        elem_a="fin_loft",
        elem_b="tail_fairing",
        reason="Fin root seats into the dorsal tail fairing.",
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
        elem_b="tail_fairing",
        reason="Stabilizer root blends into the dorsal tail fairing.",
    )
    ctx.allow_overlap(
        elevator,
        fuselage,
        elem_a="elevator_loft",
        elem_b="hull",
        reason="Elevator root passes the tail cone slot.",
    )
    ctx.allow_overlap(
        elevator,
        fuselage,
        elem_a="elevator_loft",
        elem_b="tail_fairing",
        reason="Elevator root section sits inside the faired hinge slot.",
    )
    ctx.allow_overlap(
        elevator,
        fuselage,
        elem_a="torque_tube",
        elem_b="hull",
        reason="Elevator torque tube passes through the tail cone bore.",
    )
    ctx.allow_overlap(
        elevator,
        fuselage,
        elem_a="torque_tube",
        elem_b="tail_fairing",
        reason="Elevator torque tube passes through the fairing bore.",
    )
    ctx.allow_overlap(
        elevator,
        stabilizer,
        elem_a="torque_tube",
        elem_b="stabilizer_loft",
        reason="Elevator torque tube (hinge axle) is captured inside the stabilizer trailing-edge hinge cutout.",
    )
    ctx.allow_overlap(
        rudder,
        tail_fin,
        elem_a="hinge_barrel",
        elem_b="fin_loft",
        reason="Hinge barrel is captured against the fin trailing edge (captured-pin pattern).",
    )
    ctx.allow_overlap(
        propeller,
        fuselage,
        elem_a="prop_shaft",
        elem_b="engine_face",
        reason="Propeller shaft is seated in the engine gearbox bore.",
    )
    ctx.allow_overlap(
        propeller,
        fuselage,
        elem_a="prop_shaft",
        elem_b="hull",
        reason="Propeller shaft passes through the cowl front into the engine.",
    )

    # --- joint plan: types, axes, ranges ---
    ctx.check(
        "propeller joint is continuous about +X",
        prop_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(prop_spin.axis[0] - 1.0) < 1e-6
        and prop_spin.motion_limits is not None
        and prop_spin.motion_limits.lower is None
        and prop_spin.motion_limits.upper is None,
        details=f"type={prop_spin.articulation_type}, axis={prop_spin.axis}",
    )
    rl = rudder_hinge.motion_limits
    ctx.check(
        "rudder is revolute about a near-vertical hinge, +/-30 deg",
        rudder_hinge.articulation_type == ArticulationType.REVOLUTE
        and rl is not None
        and abs(rl.lower + math.radians(30.0)) < 1e-6
        and abs(rl.upper - math.radians(30.0)) < 1e-6
        and rudder_hinge.axis[2] > 0.95
        and abs(rudder_hinge.axis[1]) < 1e-6,
        details=f"axis={rudder_hinge.axis}, limits=({rl.lower}, {rl.upper})",
    )
    el = elev_hinge.motion_limits
    ctx.check(
        "elevator is revolute about the lateral +Y axis, +/-25 deg",
        elev_hinge.articulation_type == ArticulationType.REVOLUTE
        and el is not None
        and abs(el.lower + math.radians(25.0)) < 1e-6
        and abs(el.upper - math.radians(25.0)) < 1e-6
        and abs(elev_hinge.axis[1] - 1.0) < 1e-6,
        details=f"axis={elev_hinge.axis}, limits=({el.lower}, {el.upper})",
    )

    # --- real-world scale ---
    fus_aabb = ctx.part_world_aabb(fuselage)
    prop_aabb = ctx.part_world_aabb(propeller)
    wing_aabb = ctx.part_world_aabb(wing)
    length = prop_aabb[1][0] - fus_aabb[0][0]
    ctx.check(
        "overall length about 8.5 m (spinner tip to tail)",
        8.1 < length < 8.8,
        details=f"length={length:.3f}",
    )
    span = wing_aabb[1][1] - wing_aabb[0][1]
    ctx.check(
        "wingspan about 10.5 m",
        10.3 < span < 10.7,
        details=f"span={span:.3f}",
    )

    # --- grounding: lowest geometry (blade tips at rest) grazes z ~ 0 ---
    zmin = min(
        ctx.part_world_aabb(p)[0][2]
        for p in (fuselage, wing, tail_fin, stabilizer, elevator, rudder, propeller)
    )
    ctx.check(
        "rest pose grazes ground plane",
        -0.005 <= zmin <= 0.12,
        details=f"zmin={zmin:.3f}",
    )

    # --- propeller: blades are off-axis, proving continuous rotation ---
    rest_top = prop_aabb[1][2]
    with ctx.pose({prop_spin: math.pi / 4.0}):
        spun_aabb = ctx.part_world_aabb(propeller)
        ctx.check(
            "45 deg spin swings blade tips from X- to +-pattern (off-axis proof)",
            spun_aabb[1][2] > rest_top + 0.25,
            details=f"rest_top={rest_top:.3f}, spun_top={spun_aabb[1][2]:.3f}",
        )
        ctx.check(
            "propeller disc is about 3 m across (measured at +-pattern)",
            2.8 < (spun_aabb[1][2] - spun_aabb[0][2]) < 3.2,
            details=f"disc_height={spun_aabb[1][2] - spun_aabb[0][2]:.3f}",
        )
    ctx.check(
        "spinner tip reaches the nose of the aircraft",
        prop_aabb[1][0] > 4.10,
        details=f"prop_xmax={prop_aabb[1][0]:.3f}",
    )

    # --- rudder swings the trailing edge sideways ---
    rud_rest = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_hinge: math.radians(30.0)}):
        rud_open = ctx.part_world_aabb(rudder)
        ctx.check(
            "rudder +30 deg swings trailing edge laterally",
            (rud_rest[0][1] - rud_open[0][1]) > 0.15 or (rud_open[1][1] - rud_rest[1][1]) > 0.15,
            details=f"rest_y=({rud_rest[0][1]:.3f},{rud_rest[1][1]:.3f}), open_y=({rud_open[0][1]:.3f},{rud_open[1][1]:.3f})",
        )

    # --- elevator pitches the trailing edge vertically ---
    elev_rest = ctx.part_world_aabb(elevator)
    with ctx.pose({elev_hinge: math.radians(25.0)}):
        elev_up = ctx.part_world_aabb(elevator)
        ctx.check(
            "elevator +25 deg moves trailing edge vertically",
            (elev_up[1][2] - elev_rest[1][2]) > 0.10,
            details=f"rest_zmax={elev_rest[1][2]:.3f}, up_zmax={elev_up[1][2]:.3f}",
        )

    # --- placements / seating ---
    ctx.expect_overlap(
        wing,
        fuselage,
        axes="xy",
        elem_a="wing_loft",
        elem_b="hull",
        min_overlap=0.6,
        name="wing carries through the fuselage belly",
    )
    ctx.expect_overlap(
        tail_fin,
        fuselage,
        axes="x",
        elem_a="fin_loft",
        elem_b="hull",
        min_overlap=0.3,
        name="fin root sits over the tail cone",
    )
    ctx.expect_overlap(
        elevator,
        stabilizer,
        axes="x",
        elem_a="torque_tube",
        elem_b="stabilizer_loft",
        min_overlap=0.02,
        name="elevator torque tube seats in the stabilizer hinge line",
    )
    ctx.expect_gap(
        propeller,
        fuselage,
        axis="x",
        positive_elem="spinner_shell",
        negative_elem="engine_face",
        min_gap=0.0,
        max_gap=0.06,
        name="spinner sits just ahead of the engine face",
    )
    canopy_aabb = ctx.part_element_world_aabb(fuselage, elem="canopy")
    ctx.check(
        "teardrop canopy bump rises above the fuselage spine",
        canopy_aabb is not None and canopy_aabb[1][2] > CL_Z + 0.80,
        details=f"canopy_zmax={canopy_aabb[1][2]:.3f}" if canopy_aabb else "missing",
    )
    for elem in ("side_number_left", "side_number_right"):
        aabb = ctx.part_element_world_aabb(fuselage, elem=elem)
        ctx.check(
            f"white 402 marking present on fuselage flank ({elem})",
            aabb is not None and abs((aabb[0][2] + aabb[1][2]) / 2.0 - (CL_Z + 0.05)) < 0.10,
            details=str(aabb),
        )
    for elem in ("wing_number_top", "wing_number_bottom"):
        aabb = ctx.part_element_world_aabb(wing, elem=elem)
        ctx.check(
            f"white 402 marking present on wing ({elem})",
            aabb is not None,
            details=str(aabb),
        )
    letter_aabb = ctx.part_element_world_aabb(tail_fin, elem="fin_letter_left")
    ctx.check(
        "yellow B marking sits high on the fin",
        letter_aabb is not None and letter_aabb[0][2] > CL_Z + 0.80,
        details=str(letter_aabb),
    )
    tip_aabb = ctx.part_element_world_aabb(tail_fin, elem="fin_tip")
    ctx.check(
        "fin tip cap is the top of the aircraft",
        tip_aabb is not None and tip_aabb[1][2] > CL_Z + 1.45,
        details=str(tip_aabb),
    )

    return ctx.report()


object_model = build_object_model()
