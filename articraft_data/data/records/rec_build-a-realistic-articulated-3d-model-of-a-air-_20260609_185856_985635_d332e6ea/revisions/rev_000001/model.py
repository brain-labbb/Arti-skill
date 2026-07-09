from __future__ import annotations

# Air conditioner outdoor (condenser) unit.
#
# Reference: a galvanized/stainless-steel rectangular cabinet standing on
# L-shaped mounting feet, with two axial condenser fans on the front face
# (one upper, one lower), protected by radial wire-style grille guards. Two
# U-shaped lifting/mounting brackets straddle the top. The side panels carry
# vertical louver slots, and small info/spec labels sit on the front face.
#
# Primary mechanism: the two axial condenser fans spin about their horizontal
# axis (perpendicular to the front face). They are modeled as CONTINUOUS
# articulations mounted on central hub bosses inside each fan opening.
#
# Frame: X = width (left-right), Y = depth (front face is at +Y), Z = up.
# The cabinet is centered in X; its base sits at z = 0.
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Overall dimensions (meters)
# ---------------------------------------------------------------------------
CAB_W = 0.700  # width  (X)
CAB_D = 0.300  # depth  (Y)
CAB_H = 0.820  # height (Z)
WALL = 0.006  # sheet-metal wall thickness

FRONT_Y = CAB_D / 2.0  # outer front face plane (fans live here)
BACK_Y = -CAB_D / 2.0

# Fan layout on the front face
FAN_OPENING_R = 0.150  # radius of the round opening cut in the front face
FAN_ROTOR_R = 0.140  # rotor envelope radius
FAN_HUB_R = 0.034  # central hub radius
FAN_THICK = 0.034  # rotor axial thickness
UPPER_FAN_Z = 0.560
LOWER_FAN_Z = 0.240
FAN_SPIN_Y = FRONT_Y - 0.050  # rotor center recessed behind the grille

# Grille guard (radial spokes + concentric rings) standing proud of the face
GUARD_PROUD = 0.012
GUARD_WIRE = 0.005

# Top lifting / mounting brackets
BRACKET_X = 0.190  # |X| of each bracket center from cabinet center
BRACKET_LEG_W = 0.030
BRACKET_LEG_T = 0.012
BRACKET_RISE = 0.085  # how far the inverted-U rises above the top
BRACKET_SPAN_Y = 0.230  # outer span of the U along depth

# Bottom L-shaped mounting feet (channel rails under the cabinet)
FOOT_LEN = CAB_D + 0.060
FOOT_W = 0.045
FOOT_H = 0.060
FOOT_WALL = 0.008


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------
def _ring_yspan(
    center_z: float, radius: float, wire: float, y_lo: float, y_hi: float
) -> cq.Workplane:
    """A thin annular ring in the X-Z plane spanning Y in [y_lo, y_hi].
    Centered at (0, center_z) in X-Z. (An XZ workplane extrudes toward -Y, so
    we anchor at y_hi and extrude the span.)"""
    return (
        cq.Workplane("XZ")
        .workplane(offset=-y_hi)
        .center(0.0, center_z)
        .circle(radius + wire / 2.0)
        .circle(radius - wire / 2.0)
        .extrude(y_hi - y_lo)
    )


def _disc_yspan(center_z: float, radius: float, y_lo: float, y_hi: float) -> cq.Workplane:
    """A solid disc in the X-Z plane spanning Y in [y_lo, y_hi]."""
    return (
        cq.Workplane("XZ")
        .workplane(offset=-y_hi)
        .center(0.0, center_z)
        .circle(radius)
        .extrude(y_hi - y_lo)
    )


def _grille_and_motor_features(center_z: float) -> cq.Workplane:
    """Grille guard (concentric rings + radial spokes with a center cap) plus a
    central hub boss that reaches back to the recessed fan. Built so every piece
    genuinely interpenetrates its neighbour and the outer ring embeds into the
    solid front-face rim, fusing the whole assembly into one connected solid
    with the housing shell.

    Looking in from the front: outer guard ring + spokes + center cap ->
    hub boss/shaft -> recessed fan (visible through the guard openings)."""
    r_rim = FAN_OPENING_R  # solid front face exists at radius >= r_rim
    r_out = r_rim + 0.006  # outer ring overlaps the solid rim by ~6 mm
    r_mid = r_rim * 0.60
    r_cap = FAN_HUB_R + 0.006  # center cap radius

    # Y span of the guard: embed 6 mm into the wall, stand GUARD_PROUD proud.
    y_lo = FRONT_Y - 0.006  # back face flush with inner wall (embeds into wall)
    y_hi = FRONT_Y + GUARD_PROUD
    y_mid = (y_lo + y_hi) / 2.0

    feat: cq.Workplane | None = None

    def _add(shape: cq.Workplane) -> None:
        nonlocal feat
        feat = shape if feat is None else feat.union(shape)

    # Concentric guard rings (outer ring embeds into the solid rim -> fuses shell).
    _add(_ring_yspan(center_z, r_out, 0.007, y_lo, y_hi))
    _add(_ring_yspan(center_z, r_mid, GUARD_WIRE, y_lo, y_hi))

    # Solid center cap disc the spokes converge onto.
    _add(_disc_yspan(center_z, r_cap, y_lo, y_hi))

    # Radial guard spokes from the center cap out past the outer ring; each spoke
    # spans the full guard Y depth so it fuses with both rings and the cap.
    n_spokes = 8
    r_in = r_cap - 0.004
    r_far = r_out + 0.004
    for k in range(n_spokes):
        a = 2.0 * math.pi * k / n_spokes
        dx, dz = math.cos(a), math.sin(a)
        x0, z0 = r_in * dx, r_in * dz
        x1, z1 = r_far * dx, r_far * dz
        length = math.hypot(x1 - x0, z1 - z0)
        ang = math.degrees(math.atan2(z1 - z0, x1 - x0))
        spoke = (
            cq.Workplane("XY")
            .box(length, y_hi - y_lo, GUARD_WIRE)
            .rotate((0, 0, 0), (0, 1, 0), -ang)
            .translate(((x0 + x1) / 2.0, y_mid, center_z + (z0 + z1) / 2.0))
        )
        _add(spoke)

    # Central hub boss/shaft reaching back from the center cap to the fan hub.
    # Overlaps the cap (front) and the recessed fan hub (back) -> captured fit.
    boss_lo = FAN_SPIN_Y - 0.004
    boss_hi = y_lo + 0.006  # overlaps the center cap's back region
    _add(_disc_yspan(center_z, FAN_HUB_R - 0.003, boss_lo, boss_hi))

    return feat


def _housing_shape() -> cq.Workplane:
    """Hollow steel cabinet: shelled box with two round fan openings cut into
    the front (+Y) face and vertical louver slots on both side (X) walls, then
    the grille guards and fan hub bosses unioned on so the whole front assembly
    reads as one connected steel structure."""
    outer = cq.Workplane("XY").box(
        CAB_W, CAB_D, CAB_H, centered=(True, True, False)
    )
    # Hollow it out, leaving the front face to be perforated by the fan holes.
    shell = outer.faces("<Y").shell(-WALL)

    # Round fan openings through the front face.
    for z in (UPPER_FAN_Z, LOWER_FAN_Z):
        shell = (
            shell.faces(">Y")
            .workplane(centerOption="CenterOfMass")
            .pushPoints([(0.0, z - CAB_H / 2.0)])
            .hole(2.0 * FAN_OPENING_R, depth=WALL * 3.0)
        )

    # Vertical louver slots on each side wall. Cut a column of tall thin slots.
    slot_w = 0.010
    slot_h = 0.090
    slot_gap = 0.026
    n_slots = 5
    col_z0 = 0.150
    for sign in (1.0, -1.0):
        for i in range(n_slots):
            zc = col_z0 + i * (slot_h + slot_gap) + slot_h / 2.0
            cutter = (
                cq.Workplane("XY")
                .box(WALL * 4.0, slot_w, slot_h)
                .translate((sign * CAB_W / 2.0, 0.0, zc))
            )
            shell = shell.cut(cutter)

    # Union the grille/motor features over each opening into the shell.
    for z in (UPPER_FAN_Z, LOWER_FAN_Z):
        shell = shell.union(_grille_and_motor_features(z))
    return shell


def _front_label_shape() -> cq.Workplane:
    """Two small raised rectangular spec/info plates on the front face, set off
    to the side so they clear the fan openings. Tucked against the front face."""
    # Slight back-face embed (-0.002) keeps each plate connected to the front
    # wall; the plate still stands ~3 mm proud of the face.
    plate_a = (
        cq.Workplane("XY")
        .box(0.085, 0.005, 0.055)
        .translate((0.255, FRONT_Y + 0.0005, 0.640))
    )
    plate_b = (
        cq.Workplane("XY")
        .box(0.110, 0.005, 0.050)
        .translate((0.245, FRONT_Y + 0.0005, 0.300))
    )
    return plate_a.union(plate_b)


def _bracket_shape(x_center: float) -> cq.Workplane:
    """An inverted-U lifting/mounting bracket straddling the top, with two
    feet that bolt down onto the cabinet top plus a raised cross bar."""
    top_z = CAB_H
    y_front = BRACKET_SPAN_Y / 2.0
    y_back = -BRACKET_SPAN_Y / 2.0

    def _vertical(y: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(BRACKET_LEG_W, BRACKET_LEG_T, BRACKET_RISE)
            .translate((x_center, y, top_z + BRACKET_LEG_T + BRACKET_RISE / 2.0))
        )

    # foot pads that sit flat on the top surface (bottom face touches the top).
    def _foot(y: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(BRACKET_LEG_W, 0.050, BRACKET_LEG_T)
            .translate((x_center, y, top_z + BRACKET_LEG_T / 2.0))
        )

    cross = (
        cq.Workplane("XY")
        .box(BRACKET_LEG_W, BRACKET_SPAN_Y, BRACKET_LEG_T)
        .translate(
            (x_center, 0.0, top_z + BRACKET_LEG_T + BRACKET_RISE - BRACKET_LEG_T / 2.0)
        )
    )

    bracket = (
        _vertical(y_front)
        .union(_vertical(y_back))
        .union(_foot(y_front))
        .union(_foot(y_back))
        .union(cross)
    )
    return bracket


def _feet_shape() -> cq.Workplane:
    """Two L/channel-section mounting rails under the cabinet running front-to
    back, joined by two transverse cross-ties so the base frame is one connected
    member. The rail tops contact the cabinet base."""
    xc = CAB_W / 2.0 - FOOT_W / 2.0 - 0.010
    feet: cq.Workplane | None = None
    for sign in (1.0, -1.0):
        x = sign * xc
        outer = (
            cq.Workplane("XY")
            .box(FOOT_W, FOOT_LEN, FOOT_H, centered=(True, True, False))
            .translate((x, 0.0, -FOOT_H))
        )
        # hollow the underside to read as a bent channel
        cavity = (
            cq.Workplane("XY")
            .box(FOOT_W - 2.0 * FOOT_WALL, FOOT_LEN + 0.02, FOOT_H - FOOT_WALL,
                 centered=(True, True, False))
            .translate((x, 0.0, -FOOT_H))
        )
        rail = outer.cut(cavity)
        feet = rail if feet is None else feet.union(rail)

    # Transverse cross-ties bridging the two rails (front and rear), top flush
    # with the rail tops so the whole base frame is a single connected member.
    tie_len = 2.0 * xc + FOOT_W
    for y in (CAB_D / 2.0 - 0.030, -(CAB_D / 2.0 - 0.030)):
        tie = (
            cq.Workplane("XY")
            .box(tie_len, FOOT_W, FOOT_WALL, centered=(True, True, False))
            .translate((0.0, y, -FOOT_WALL))
        )
        feet = feet.union(tie)
    return feet


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ac_outdoor_unit")

    steel = model.material("galv_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    guard_mat = model.material("guard_chrome", rgba=(0.55, 0.57, 0.60, 1.0))
    label_mat = model.material("spec_label", rgba=(0.88, 0.88, 0.84, 1.0))
    fan_mat = model.material("fan_blade", rgba=(0.30, 0.31, 0.33, 1.0))

    # --- Housing (root) ---
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell.obj"),
        material=steel,
        name="housing_shell",
    )
    # Raised spec/info plates, flush against the front face (their own material).
    housing.visual(
        mesh_from_cadquery(_front_label_shape(), "front_labels.obj"),
        material=label_mat,
        name="front_labels",
    )
    # Note: the two grille guards, motor spiders, and fan hub bosses are folded
    # into the `housing_shell` mesh so the whole front assembly is one connected
    # steel structure (see `_grille_and_motor_features`).
    _ = guard_mat
    housing.inertial = Inertial.from_geometry(
        Box((CAB_W, CAB_D, CAB_H)), mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, CAB_H / 2.0)),
    )

    # --- Top lifting brackets (fixed) ---
    left_bracket = model.part("left_bracket")
    left_bracket.visual(
        mesh_from_cadquery(_bracket_shape(-BRACKET_X), "left_bracket.obj"),
        material=dark_steel,
        name="left_bracket",
    )
    left_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_LEG_W, BRACKET_SPAN_Y, BRACKET_RISE)), mass=0.6,
        origin=Origin(xyz=(-BRACKET_X, 0.0, CAB_H + BRACKET_RISE / 2.0)),
    )

    right_bracket = model.part("right_bracket")
    right_bracket.visual(
        mesh_from_cadquery(_bracket_shape(BRACKET_X), "right_bracket.obj"),
        material=dark_steel,
        name="right_bracket",
    )
    right_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_LEG_W, BRACKET_SPAN_Y, BRACKET_RISE)), mass=0.6,
        origin=Origin(xyz=(BRACKET_X, 0.0, CAB_H + BRACKET_RISE / 2.0)),
    )

    # --- Bottom mounting feet (fixed) ---
    feet = model.part("mounting_feet")
    feet.visual(
        mesh_from_cadquery(_feet_shape(), "mounting_feet.obj"),
        material=dark_steel,
        name="mounting_feet",
    )
    feet.inertial = Inertial.from_geometry(
        Box((CAB_W, FOOT_LEN, FOOT_H)), mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -FOOT_H / 2.0)),
    )

    # --- Fans (rotating) ---
    # Rotor authored about local Z (FanRotorGeometry spins about Z); we mount it
    # so its spin axis is world Y by orienting the joint axis along Y and
    # rotating the rotor mesh visual to face +Y.
    def _make_fan(name: str, mesh_name: str):
        part = model.part(name)
        rotor = FanRotorGeometry(
            FAN_ROTOR_R,
            FAN_HUB_R,
            7,
            thickness=FAN_THICK,
            blade_pitch_deg=26.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="broad", camber=0.10),
            hub=FanRotorHub(style="domed"),
        )
        part.visual(
            mesh_from_geometry(rotor, mesh_name),
            material=fan_mat,
            # Rotor is built about +Z; rotate -90deg about X so it faces +Y.
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            name="fan_rotor",
        )
        part.inertial = Inertial.from_geometry(
            Cylinder(radius=FAN_ROTOR_R, length=FAN_THICK), mass=0.9,
        )
        return part

    upper_fan = _make_fan("upper_fan", "upper_fan_rotor.obj")
    lower_fan = _make_fan("lower_fan", "lower_fan_rotor.obj")

    # Fans spin about the world-Y axis (perpendicular to the front face).
    model.articulation(
        "upper_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=upper_fan,
        origin=Origin(xyz=(0.0, FAN_SPIN_Y, UPPER_FAN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )
    model.articulation(
        "lower_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=lower_fan,
        origin=Origin(xyz=(0.0, FAN_SPIN_Y, LOWER_FAN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    # Brackets and feet are rigidly fixed to the housing.
    model.articulation(
        "housing_to_left_bracket",
        ArticulationType.FIXED,
        parent=housing,
        child=left_bracket,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_right_bracket",
        ArticulationType.FIXED,
        parent=housing,
        child=right_bracket,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_feet",
        ArticulationType.FIXED,
        parent=housing,
        child=feet,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    upper_fan = object_model.get_part("upper_fan")
    lower_fan = object_model.get_part("lower_fan")
    left_bracket = object_model.get_part("left_bracket")
    right_bracket = object_model.get_part("right_bracket")
    feet = object_model.get_part("mounting_feet")

    upper_spin = object_model.get_articulation("upper_fan_spin")
    lower_spin = object_model.get_articulation("lower_fan_spin")

    # The fan hubs are intentionally captured on the front-face hub bosses that
    # are folded into the housing shell (seated/captured fit), so the rotor and
    # the housing front structure are allowed to interpenetrate locally.
    upper_rotor = upper_fan.get_visual("fan_rotor")
    lower_rotor = lower_fan.get_visual("fan_rotor")
    housing_shell = housing.get_visual("housing_shell")
    ctx.allow_overlap(
        upper_fan, housing,
        elem_a="fan_rotor", elem_b="housing_shell",
        reason="Upper fan hub is captured on the housing front hub boss (seated shaft fit).",
    )
    ctx.allow_overlap(
        lower_fan, housing,
        elem_a="fan_rotor", elem_b="housing_shell",
        reason="Lower fan hub is captured on the housing front hub boss (seated shaft fit).",
    )

    # --- Mechanism: two continuous axial fans spinning about world Y ---
    for joint, label in ((upper_spin, "upper"), (lower_spin, "lower")):
        ctx.check(
            f"{label}_fan_is_continuous",
            str(joint.articulation_type).endswith("CONTINUOUS"),
            f"{label} fan must be a CONTINUOUS rotation joint.",
        )
        ax = tuple(round(float(v), 6) for v in joint.axis)
        ctx.check(
            f"{label}_fan_axis_is_y",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-3 and abs(ax[2]) < 1e-3,
            f"{label} fan spin axis must be world Y, got {ax!r}.",
        )

    # --- Hero geometry present and placed ---
    # Two fan rotors exist, sized ~ the opening diameter, on the front (+Y) face.
    for fan, label, zc in (
        (upper_fan, "upper", UPPER_FAN_Z),
        (lower_fan, "lower", LOWER_FAN_Z),
    ):
        aabb = ctx.part_world_aabb(fan)
        ctx.check(f"{label}_fan_aabb", aabb is not None, "Fan AABB missing.")
        if aabb is not None:
            mins, maxs = aabb
            dia_x = float(maxs[0] - mins[0])
            dia_z = float(maxs[2] - mins[2])
            ctx.check(
                f"{label}_fan_diameter",
                abs(dia_x - 2.0 * FAN_ROTOR_R) <= 0.03
                and abs(dia_z - 2.0 * FAN_ROTOR_R) <= 0.03,
                f"dia_x={dia_x:.3f} dia_z={dia_z:.3f}",
            )
            zc_world = 0.5 * (mins[2] + maxs[2])
            ctx.check(
                f"{label}_fan_height",
                abs(zc_world - zc) <= 0.02,
                f"fan center z={zc_world:.3f}, expected {zc:.3f}",
            )
            # Fan is shallow along depth (axial thickness much less than diameter).
            depth_y = float(maxs[1] - mins[1])
            ctx.check(
                f"{label}_fan_axial_thin",
                depth_y < 0.5 * (dia_x),
                f"fan depth_y={depth_y:.3f} not axial-thin",
            )

    # Fans sit at the front of the housing (their center beyond housing mid-depth).
    h_aabb = ctx.part_world_aabb(housing)
    if h_aabb is not None:
        u_pos = ctx.part_world_position(upper_fan)
        l_pos = ctx.part_world_position(lower_fan)
        ctx.check(
            "fans_on_front_face",
            u_pos is not None and l_pos is not None
            and u_pos[1] > 0.0 and l_pos[1] > 0.0,
            f"upper={u_pos}, lower={l_pos}",
        )

    # --- Fan rotor hub is seated on the housing hub boss (not floating) ---
    ctx.expect_contact(
        upper_fan, housing,
        elem_a="fan_rotor", elem_b="housing_shell",
        name="upper_fan_seated_on_boss",
    )
    ctx.expect_contact(
        lower_fan, housing,
        elem_a="fan_rotor", elem_b="housing_shell",
        name="lower_fan_seated_on_boss",
    )

    # --- Fan actually rotates when posed (blades move off-axis) ---
    blade_aabb_rest = ctx.part_element_world_aabb(upper_fan, elem="fan_rotor")
    with ctx.pose({upper_spin: math.pi / 2.0}):
        blade_aabb_spun = ctx.part_element_world_aabb(upper_fan, elem="fan_rotor")
    ctx.check(
        "upper_fan_rotation_moves_blades",
        blade_aabb_rest is not None and blade_aabb_spun is not None
        and (
            abs(blade_aabb_rest[0][0] - blade_aabb_spun[0][0]) > 1e-4
            or abs(blade_aabb_rest[0][2] - blade_aabb_spun[0][2]) > 1e-4
        ),
        "Rotating the upper fan by 90deg should move its blades.",
    )

    # --- Hero housing features present (grille guards stand proud of the face) ---
    # The grille guards are folded into the housing_shell mesh and stand proud
    # of the front face; verify the shell extends beyond the cabinet box front.
    sh_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    ctx.check(
        "grille_guards_proud_of_face",
        sh_aabb is not None and sh_aabb[1][1] >= FRONT_Y + GUARD_PROUD * 0.5,
        f"housing front extent {None if sh_aabb is None else sh_aabb[1][1]:.4f} "
        f"should exceed front face {FRONT_Y:.4f} by the guard stand-off.",
    )
    ctx.check(
        "housing_has_labels",
        housing.get_visual("front_labels") is not None,
        "Front spec labels must be present.",
    )
    _ = (upper_rotor, lower_rotor, housing_shell)

    # --- Supporting structure: brackets on top, feet on bottom ---
    for bracket, label in ((left_bracket, "left"), (right_bracket, "right")):
        b_aabb = ctx.part_world_aabb(bracket)
        ctx.check(
            f"{label}_bracket_above_top",
            b_aabb is not None and b_aabb[1][2] > CAB_H + BRACKET_RISE * 0.5,
            f"{label} bracket should rise above the cabinet top.",
        )
        # Bracket feet must contact the cabinet top (no floating bracket).
        ctx.expect_gap(
            bracket, housing, axis="z",
            max_gap=0.002, max_penetration=0.004,
            name=f"{label}_bracket_seats_on_top",
        )

    f_aabb = ctx.part_world_aabb(feet)
    ctx.check(
        "feet_below_cabinet",
        f_aabb is not None and f_aabb[0][2] < -0.01,
        "Mounting feet should sit below the cabinet base.",
    )
    ctx.expect_contact(
        feet, housing, name="feet_contact_housing",
    )

    return ctx.report()


object_model = build_object_model()
