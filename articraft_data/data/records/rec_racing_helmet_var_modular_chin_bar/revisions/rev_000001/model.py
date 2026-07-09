from __future__ import annotations

# Racing helmet with a modular drop-down chin bar and flip-up clear visor.
#
# Real object: a glossy red full-face racing helmet with a black chin bar,
# clear visor, and a modular chin bar mechanism. The chin bar pivots on visible
# revolute hinges at the sides (near the jawline) and swings downward to expose
# the lower face. The visor independently flips up
# and back over the crown on temple pivot studs.
#
# Build strategy:
# - Shell: a thick-walled rounded dome (CadQuery ellipsoid shelled and
#   trimmed) with front eye-port and chin-bar openings cut into it. Glossy red.
# - Chin bar: a separate curved shell piece that fills the chin opening,
#   with hinge arms extending to pivot studs on the shell sides. Black.
# - Visor: a thin curved clear panel wrapping the eye port, with side arms
#   riding on temple pivot studs.
# - Two articulations: visor hinge (temple pivot) and chin bar hinge (jaw pivot).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Geometry constants (meters). Geometry is authored about the head center;
# +X forward (face), +Z up. HEAD_CZ lifts everything so the shell's neck rim
# rests on the ground plane (world z-min ~= 0).
# ---------------------------------------------------------------------------
HEAD_RX = 0.125  # shell half-extent, front/back
HEAD_RY = 0.100  # shell half-extent, left/right
HEAD_RZ = 0.110  # shell half-extent, vertical (crown)
SHELL_WALL = 0.012

# Neck plane: the full ellipsoid is trimmed flat here, opening the cavity at
# the bottom so the helmet has a real neck opening and a flat resting rim.
NECK_Z = -0.085
HEAD_CZ = -NECK_Z  # world height of the head center (rim sits on the ground)

# Visor pivot location: near the temples, at the top edge of the eye port.
PIVOT_X = 0.055
PIVOT_Z = 0.055

# Eye port opening (front of shell), measured from head center.
PORT_Z_LO = -0.012
PORT_Z_HI = 0.055

# Eye-port cut box: punches through the front wall only.
PORT_CUT_X_LO = 0.04
PORT_CUT_X_HI = 0.18
PORT_CUT_Y_HALF = 0.075

# Visor side-edge relief: removes visor material near the pivot so the raised
# visor clears the shell side walls.
VISOR_SIDE_CUT_X = 0.08
VISOR_SIDE_CUT_Z = 0.035

# Visor side arms: flat capsule plates linking visor to temple pivots.
ARM_ANCHOR_X = 0.085
ARM_ANCHOR_Z = 0.025
ARM_R = 0.013
ARM_Y_IN = 0.084
ARM_T = 0.008

# Visor hinge pin (stud) along Y.
STUD_RADIUS = 0.011
STUD_LEN = 0.035
STUD_Y_CENTER = 0.0775

# ---------------------------------------------------------------------------
# Chin bar hinge: at the sides of the helmet near the jawline, below the
# visor pivot. The chin bar swings downward on the same left-right hinge line.
# Pivot is positioned low enough that the arm end caps clear the visor panel.
# ---------------------------------------------------------------------------
CHIN_PIVOT_X = 0.045
CHIN_PIVOT_Z = -0.040

# Chin bar cutout in shell: removes the front center portion below the eye
# port, leaving the side walls intact for hinge mounting.
CHIN_CUT_X_LO = 0.018
CHIN_CUT_Y_HALF = 0.076

# Chin bar hinge arms: flat capsule plates linking chin bar to jaw pivots.
# Arms overlap slightly with the body (CHIN_ARM_Y_IN < body y_half) so the
# union creates a single connected mesh.
CHIN_ARM_ANCHOR_X = 0.068
CHIN_ARM_ANCHOR_Z = -0.038
CHIN_ARM_R = 0.016  # larger end cap at pivot for visible hinge knuckle
CHIN_ARM_Y_IN = 0.073  # inner face overlaps with body edge at y=0.074
CHIN_ARM_T = 0.012

# Chin bar hinge pin (stud) along Y.
CHIN_STUD_RADIUS = 0.011
CHIN_STUD_LEN = 0.032
CHIN_STUD_Y_CENTER = 0.080  # centered on arm: y = 0.073 to 0.085


def _full_ellipsoid(rx: float, ry: float, rz: float) -> cq.Solid:
    """Full (not hemispherical) ellipsoid centered at the origin."""
    sphere = cq.Solid.makeSphere(1.0, angleDegrees1=-90, angleDegrees2=90)
    matrix = cq.Matrix(
        [
            [rx, 0.0, 0.0, 0.0],
            [0.0, ry, 0.0, 0.0],
            [0.0, 0.0, rz, 0.0],
        ]
    )
    return sphere.transformGeometry(matrix)


def _build_shell() -> cq.Workplane:
    """Glossy red helmet shell: thick-walled ellipsoid trimmed at the neck,
    with eye-port and chin-bar openings cut through the front wall."""
    outer_ell = _full_ellipsoid(HEAD_RX, HEAD_RY, HEAD_RZ)
    inner_ell = _full_ellipsoid(
        HEAD_RX - SHELL_WALL, HEAD_RY - SHELL_WALL, HEAD_RZ - SHELL_WALL
    )
    shell = cq.Workplane(obj=outer_ell).cut(cq.Workplane(obj=inner_ell))

    # Trim away everything below the neck plane.
    neck_cut = (
        cq.Workplane("XY").box(0.5, 0.5, 0.3).translate((0.0, 0.0, NECK_Z - 0.15))
    )
    shell = shell.cut(neck_cut)

    # Carve the front eye-port window.
    port_z_center = (PORT_Z_LO + PORT_Z_HI) / 2.0
    port_h = PORT_Z_HI - PORT_Z_LO
    port_x_center = (PORT_CUT_X_LO + PORT_CUT_X_HI) / 2.0
    port_cut = (
        cq.Workplane("XY")
        .box(PORT_CUT_X_HI - PORT_CUT_X_LO, 2.0 * PORT_CUT_Y_HALF, port_h)
        .edges("|Z")
        .fillet(0.030)
        .translate((port_x_center, 0.0, port_z_center))
    )
    shell = shell.cut(port_cut)

    # Chin bar opening: remove the front center portion below the eye port,
    # leaving the side walls for chin-bar hinge mounting.
    chin_cut_z_lo = NECK_Z - 0.005
    chin_cut_z_hi = PORT_Z_LO + 0.002
    chin_cut = (
        cq.Workplane("XY")
        .box(0.20, 2.0 * CHIN_CUT_Y_HALF, chin_cut_z_hi - chin_cut_z_lo)
        .translate(
            (
                CHIN_CUT_X_LO + 0.10,
                0.0,
                (chin_cut_z_lo + chin_cut_z_hi) / 2.0,
            )
        )
    )
    shell = shell.cut(chin_cut)

    return shell


def _build_visor() -> cq.Workplane:
    """Thin curved clear visor panel wrapping the front eye port."""
    rx = HEAD_RX + 0.014
    ry = HEAD_RY + 0.014
    rz = HEAD_RZ + 0.014
    wall = 0.004

    outer = _full_ellipsoid(rx, ry, rz)
    inner = _full_ellipsoid(rx - wall, ry - wall, rz - wall)
    visor_shell = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))

    z_lo = PORT_Z_LO - 0.001
    z_hi = PORT_Z_HI + 0.010
    z_center = (z_lo + z_hi) / 2.0
    z_h = z_hi - z_lo
    keep = (
        cq.Workplane("XY")
        .box(0.22, 0.26, z_h)
        .edges("|X")
        .fillet(0.030)
        .translate((0.16, 0.0, z_center))
    )
    visor_panel = visor_shell.intersect(keep)
    visor_panel = _cut_visor_side_relief(visor_panel)
    return visor_panel


def _cut_visor_side_relief(panel: cq.Workplane) -> cq.Workplane:
    """Remove side-wrap material above the pivot line so the raised visor
    clears the shell side walls."""
    relief = (
        cq.Workplane("XY")
        .box(0.30, 0.30, 0.20)
        .translate((VISOR_SIDE_CUT_X - 0.15, 0.0, VISOR_SIDE_CUT_Z + 0.10))
    )
    return panel.cut(relief)


def _build_visor_arm(side: float) -> cq.Workplane:
    """Flat capsule plate linking the visor band to the temple pivot."""
    ux = ARM_ANCHOR_X - PIVOT_X
    uz = ARM_ANCHOR_Z - PIVOT_Z
    length = math.hypot(ux, uz)
    ux, uz = ux / length, uz / length
    px, pz = -uz * ARM_R, ux * ARM_R
    bar = (
        cq.Workplane("XZ")
        .moveTo(PIVOT_X + px, PIVOT_Z + pz)
        .lineTo(ARM_ANCHOR_X + px, ARM_ANCHOR_Z + pz)
        .threePointArc(
            (ARM_ANCHOR_X + ux * ARM_R, ARM_ANCHOR_Z + uz * ARM_R),
            (ARM_ANCHOR_X - px, ARM_ANCHOR_Z - pz),
        )
        .lineTo(PIVOT_X - px, PIVOT_Z - pz)
        .threePointArc(
            (PIVOT_X - ux * ARM_R, PIVOT_Z - uz * ARM_R),
            (PIVOT_X + px, PIVOT_Z + pz),
        )
        .close()
        .extrude(ARM_T)
    )
    shift = (ARM_Y_IN + ARM_T) if side > 0 else -ARM_Y_IN
    return bar.translate((0.0, shift, 0.0))


def _build_visor_arms() -> cq.Workplane:
    return _build_visor_arm(1.0).union(_build_visor_arm(-1.0))


def _build_visor_trim() -> cq.Workplane:
    """Black trim band along the top edge of the visor."""
    rx = HEAD_RX + 0.016
    ry = HEAD_RY + 0.016
    rz = HEAD_RZ + 0.016
    wall = 0.0055
    outer = _full_ellipsoid(rx, ry, rz)
    inner = _full_ellipsoid(rx - wall, ry - wall, rz - wall)
    band = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))
    z_center = PORT_Z_HI + 0.001
    keep = (
        cq.Workplane("XY")
        .box(0.22, 0.26, 0.012)
        .translate((0.16, 0.0, z_center))
    )
    return _cut_visor_side_relief(band.intersect(keep))


# ---------------------------------------------------------------------------
# Chin bar geometry: the modular flip-up lower face guard.
# ---------------------------------------------------------------------------

def _build_chin_bar_body() -> cq.Workplane:
    """Chin bar shell: curved wall matching the helmet contour, filling the
    chin-bar opening in the main shell with small gaps at the seams."""
    outer = _full_ellipsoid(
        HEAD_RX + 0.002, HEAD_RY + 0.002, HEAD_RZ + 0.002
    )
    inner = _full_ellipsoid(
        HEAD_RX - SHELL_WALL + 0.003,
        HEAD_RY - SHELL_WALL + 0.003,
        HEAD_RZ - SHELL_WALL + 0.003,
    )
    wall = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))

    # Chin bar region with small seam gaps. Keep only the front portion
    # (x > CHIN_PIVOT_X) so the chin bar doesn't extend behind the pivot.
    # This ensures the entire chin bar swings up and forward when flipped.
    z_lo = NECK_Z + 0.003
    z_hi = PORT_Z_LO - 0.002
    y_half = 0.074
    keep = (
        cq.Workplane("XY")
        .box(0.20, 2.0 * y_half, z_hi - z_lo)
        .edges("|Z")
        .fillet(0.020)
        .translate((CHIN_PIVOT_X + 0.10, 0.0, (z_lo + z_hi) / 2.0))
    )
    return wall.intersect(keep)


def _build_chin_bar_arm(side: float) -> cq.Workplane:
    """Flat capsule plate linking the chin bar body to the jaw pivot on one
    side (+1 left / -1 right), authored in the head-centered frame."""
    ux = CHIN_ARM_ANCHOR_X - CHIN_PIVOT_X
    uz = CHIN_ARM_ANCHOR_Z - CHIN_PIVOT_Z
    length = math.hypot(ux, uz)
    ux, uz = ux / length, uz / length
    px, pz = -uz * CHIN_ARM_R, ux * CHIN_ARM_R
    bar = (
        cq.Workplane("XZ")
        .moveTo(CHIN_PIVOT_X + px, CHIN_PIVOT_Z + pz)
        .lineTo(CHIN_ARM_ANCHOR_X + px, CHIN_ARM_ANCHOR_Z + pz)
        .threePointArc(
            (CHIN_ARM_ANCHOR_X + ux * CHIN_ARM_R, CHIN_ARM_ANCHOR_Z + uz * CHIN_ARM_R),
            (CHIN_ARM_ANCHOR_X - px, CHIN_ARM_ANCHOR_Z - pz),
        )
        .lineTo(CHIN_PIVOT_X - px, CHIN_PIVOT_Z - pz)
        .threePointArc(
            (CHIN_PIVOT_X - ux * CHIN_ARM_R, CHIN_PIVOT_Z - uz * CHIN_ARM_R),
            (CHIN_PIVOT_X + px, CHIN_PIVOT_Z + pz),
        )
        .close()
        .extrude(CHIN_ARM_T)
    )
    shift = (CHIN_ARM_Y_IN + CHIN_ARM_T) if side > 0 else -CHIN_ARM_Y_IN
    return bar.translate((0.0, shift, 0.0))


def _build_chin_bar_arms() -> cq.Workplane:
    return _build_chin_bar_arm(1.0).union(_build_chin_bar_arm(-1.0))


def _build_chin_bar() -> cq.Workplane:
    """Complete chin bar: body + hinge arms unioned into a single connected
    mesh. The arm inner faces overlap with the body side edges so the boolean
    union produces one connected solid. The arm's large circular end cap at
    the pivot reads as the visible hinge knuckle."""
    body = _build_chin_bar_body()
    arms = _build_chin_bar_arms()
    return body.union(arms)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_helmet_modular_chinbar")

    red_gloss = model.material("shell_red", rgba=(0.78, 0.05, 0.06, 1.0))
    trim_black = model.material("trim_black", rgba=(0.06, 0.06, 0.07, 1.0))
    visor_clear = model.material("visor_clear", rgba=(0.12, 0.13, 0.16, 0.45))
    stud_black = model.material("stud_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_dark = model.material("hinge_gunmetal", rgba=(0.18, 0.18, 0.20, 1.0))

    # ----- Shell (root) -----
    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_build_shell(), "shell"),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=red_gloss,
        name="shell_dome",
    )
    shell.inertial = Inertial.from_geometry(
        Cylinder(radius=HEAD_RY, length=2.0 * HEAD_RZ),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
    )

    # ----- Visor pivot studs (mounted on shell, define visor hinge line) -----
    for side, sy in (("left", 1.0), ("right", -1.0)):
        stud = model.part(f"visor_pivot_{side}")
        stud.visual(
            Cylinder(radius=STUD_RADIUS, length=STUD_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stud_black,
            name=f"visor_pivot_{side}",
        )
        stud.inertial = Inertial.from_geometry(
            Cylinder(radius=STUD_RADIUS, length=STUD_LEN), mass=0.02
        )
        model.articulation(
            f"shell_to_visor_pivot_{side}",
            ArticulationType.FIXED,
            parent=shell,
            child=stud,
            origin=Origin(xyz=(PIVOT_X, sy * STUD_Y_CENTER, PIVOT_Z + HEAD_CZ)),
        )

    # ----- Chin bar pivot studs (mounted on shell, define chin hinge line) -----
    for side, sy in (("left", 1.0), ("right", -1.0)):
        stud = model.part(f"chin_pivot_{side}")
        stud.visual(
            Cylinder(radius=CHIN_STUD_RADIUS, length=CHIN_STUD_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stud_black,
            name=f"chin_pivot_{side}",
        )
        stud.inertial = Inertial.from_geometry(
            Cylinder(radius=CHIN_STUD_RADIUS, length=CHIN_STUD_LEN), mass=0.02
        )
        model.articulation(
            f"shell_to_chin_pivot_{side}",
            ArticulationType.FIXED,
            parent=shell,
            child=stud,
            origin=Origin(
                xyz=(CHIN_PIVOT_X, sy * CHIN_STUD_Y_CENTER, CHIN_PIVOT_Z + HEAD_CZ)
            ),
        )

    # ----- Visor (clear flip-up panel) -----
    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_build_visor(), "visor"),
        origin=Origin(xyz=(-PIVOT_X, 0.0, -PIVOT_Z)),
        material=visor_clear,
        name="visor_panel",
    )
    visor.visual(
        mesh_from_cadquery(_build_visor_trim(), "visor_trim"),
        origin=Origin(xyz=(-PIVOT_X, 0.0, -PIVOT_Z)),
        material=trim_black,
        name="visor_trim",
    )
    visor.visual(
        mesh_from_cadquery(_build_visor_arms(), "visor_arms"),
        origin=Origin(xyz=(-PIVOT_X, 0.0, -PIVOT_Z)),
        material=stud_black,
        name="visor_pivot_arms",
    )
    visor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.10),
        mass=0.12,
        origin=Origin(xyz=(-PIVOT_X + 0.10, 0.0, -PIVOT_Z + 0.02)),
    )

    # Visor revolute hinge: axis along Y. Positive q about -Y flips the visor
    # up and back over the crown.
    model.articulation(
        "shell_to_visor",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=visor,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z + HEAD_CZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(95.0)
        ),
    )

    # ----- Chin bar (modular flip-up lower face guard) -----
    chin_bar = model.part("chin_bar")
    chin_bar.visual(
        mesh_from_cadquery(_build_chin_bar(), "chin_bar"),
        origin=Origin(xyz=(-CHIN_PIVOT_X, 0.0, -CHIN_PIVOT_Z)),
        material=trim_black,
        name="chin_bar_shell",
    )
    chin_bar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=0.08),
        mass=0.18,
        origin=Origin(
            xyz=(-CHIN_PIVOT_X + 0.08, 0.0, -CHIN_PIVOT_Z - 0.03)
        ),
    )

    # Chin bar revolute hinge: same left-right hinge line as before, with the
    # sign reversed so positive q swings the lower guard downward instead of
    # up over the shell.
    model.articulation(
        "shell_to_chin_bar",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=chin_bar,
        origin=Origin(xyz=(CHIN_PIVOT_X, 0.0, CHIN_PIVOT_Z + HEAD_CZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.radians(65.0)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    visor = object_model.get_part("visor")
    chin_bar = object_model.get_part("chin_bar")

    visor_stud_l = object_model.get_part("visor_pivot_left")
    visor_stud_r = object_model.get_part("visor_pivot_right")
    chin_stud_l = object_model.get_part("chin_pivot_left")
    chin_stud_r = object_model.get_part("chin_pivot_right")

    visor_hinge = object_model.get_articulation("shell_to_visor")
    chin_hinge = object_model.get_articulation("shell_to_chin_bar")

    # --- Allow intentional overlaps for hinge captures ---

    # Visor pivot studs embed into the shell wall as hinge bosses.
    for stud, side in ((visor_stud_l, "left"), (visor_stud_r, "right")):
        ctx.allow_overlap(
            stud,
            shell,
            reason=f"Visor {side} pivot stud inner end is seated into the shell wall as a hinge boss.",
        )
    # Visor side arms ride on the temple studs (hinge capture).
    for stud, side in ((visor_stud_l, "left"), (visor_stud_r, "right")):
        ctx.allow_overlap(
            stud,
            visor,
            reason=f"Visor {side} side arm is pinned on the temple stud it rotates about.",
        )

    # Chin pivot studs embed into the shell wall as hinge bosses.
    for stud, side in ((chin_stud_l, "left"), (chin_stud_r, "right")):
        ctx.allow_overlap(
            stud,
            shell,
            reason=f"Chin bar {side} pivot stud inner end is seated into the shell wall as a hinge boss.",
        )
    # Chin bar hinge arms ride on the jaw studs (hinge capture).
    for stud, side in ((chin_stud_l, "left"), (chin_stud_r, "right")):
        ctx.allow_overlap(
            stud,
            chin_bar,
            reason=f"Chin bar {side} hinge arm is pinned on the jaw pivot stud it rotates about.",
        )
    # Chin bar hinge arms pass through the shell side wall to reach the
    # externally mounted pivot studs (realistic hinge capture through the shell).
    ctx.allow_overlap(
        chin_bar,
        shell,
        reason="Chin bar hinge arms pass through the shell side wall to reach the pivot studs, representing a realistic captured hinge mechanism.",
    )

    # --- Visor hinge checks ---
    ctx.check(
        "visor hinge is revolute",
        visor_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {visor_hinge.articulation_type}",
    )
    vax = visor_hinge.axis
    ctx.check(
        "visor hinge axis is left-right (Y)",
        abs(vax[1]) > 0.9 and abs(vax[0]) < 0.1 and abs(vax[2]) < 0.1,
        details=f"axis={vax}",
    )

    # Visor pivot studs anchored to shell.
    ctx.expect_contact(visor_stud_l, shell, name="left visor pivot anchored to shell")
    ctx.expect_contact(visor_stud_r, shell, name="right visor pivot anchored to shell")

    # Closed visor covers the eye port.
    with ctx.pose({visor_hinge: 0.0}):
        ctx.expect_origin_gap(
            visor, shell, axis="x", min_gap=0.05,
            name="closed visor sits in front of shell center",
        )
        ctx.expect_overlap(
            visor, shell, axes="z", min_overlap=0.04,
            name="closed visor covers eye-port height",
        )
        ctx.expect_contact(
            visor, visor_stud_l, name="closed visor arm rides the left pivot stud"
        )
        ctx.expect_contact(
            visor, visor_stud_r, name="closed visor arm rides the right pivot stud"
        )
        visor_closed_aabb = ctx.part_world_aabb(visor)

    # Raised visor flips up over the crown.
    with ctx.pose({visor_hinge: math.radians(95.0)}):
        visor_open_aabb = ctx.part_world_aabb(visor)
        ctx.expect_contact(
            visor, visor_stud_l, name="raised visor still held by the left pivot stud"
        )
        ctx.expect_contact(
            visor, visor_stud_r, name="raised visor still held by the right pivot stud"
        )

    ctx.check(
        "raised visor lifts upward over the crown",
        visor_closed_aabb is not None
        and visor_open_aabb is not None
        and visor_open_aabb[1][2] > visor_closed_aabb[1][2] + 0.012,
        details=(
            f"closed top z={visor_closed_aabb[1][2] if visor_closed_aabb else None}, "
            f"open top z={visor_open_aabb[1][2] if visor_open_aabb else None}"
        ),
    )
    ctx.check(
        "raised visor retracts back over the crown",
        visor_closed_aabb is not None
        and visor_open_aabb is not None
        and visor_open_aabb[1][0] < visor_closed_aabb[1][0] - 0.015,
        details=(
            f"closed front x={visor_closed_aabb[1][0] if visor_closed_aabb else None}, "
            f"open front x={visor_open_aabb[1][0] if visor_open_aabb else None}"
        ),
    )

    # --- Chin bar hinge checks ---
    ctx.check(
        "chin bar hinge is revolute",
        chin_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {chin_hinge.articulation_type}",
    )
    cax = chin_hinge.axis
    ctx.check(
        "chin bar hinge axis is left-right (Y)",
        abs(cax[1]) > 0.9 and abs(cax[0]) < 0.1 and abs(cax[2]) < 0.1,
        details=f"axis={cax}",
    )

    # Chin bar hinge origin is at the jaw pivot, lower than the visor pivot.
    chin_origin = chin_hinge.origin
    visor_origin = visor_hinge.origin
    ctx.check(
        "chin bar hinge is below visor hinge",
        chin_origin.xyz[2] < visor_origin.xyz[2] - 0.02,
        details=f"chin_z={chin_origin.xyz[2]}, visor_z={visor_origin.xyz[2]}",
    )

    # Chin pivot studs anchored to shell.
    ctx.expect_contact(chin_stud_l, shell, name="left chin pivot anchored to shell")
    ctx.expect_contact(chin_stud_r, shell, name="right chin pivot anchored to shell")

    # Chin pivot studs straddle the head on opposite sides.
    ctx.expect_origin_distance(
        chin_stud_l, chin_stud_r, axes="y", min_dist=0.12,
        name="chin pivot studs straddle the head",
    )

    # Closed chin bar covers the lower face opening.
    with ctx.pose({chin_hinge: 0.0}):
        # Chin bar is forward of the shell center.
        ctx.expect_origin_gap(
            chin_bar, shell, axis="x", min_gap=0.01,
            name="closed chin bar sits in front of shell center",
        )
        # Chin bar spans the lower face region vertically.
        ctx.expect_overlap(
            chin_bar, shell, axes="z", min_overlap=0.02,
            name="closed chin bar covers lower face height",
        )
        # Chin bar hinge arms ride on the jaw pivot studs.
        ctx.expect_contact(
            chin_bar, chin_stud_l,
            name="closed chin bar arm rides the left jaw pivot stud",
        )
        ctx.expect_contact(
            chin_bar, chin_stud_r,
            name="closed chin bar arm rides the right jaw pivot stud",
        )
        chin_closed_aabb = ctx.part_world_aabb(chin_bar)

    # Open chin bar swings downward on the same jaw hinge, exposing the lower
    # face without sweeping up through the helmet shell.
    with ctx.pose({chin_hinge: math.radians(65.0)}):
        chin_open_aabb = ctx.part_world_aabb(chin_bar)
        # Still held by both jaw pivot studs.
        ctx.expect_contact(
            chin_bar, chin_stud_l,
            name="lowered chin bar still held by the left jaw pivot stud",
        )
        ctx.expect_contact(
            chin_bar, chin_stud_r,
            name="lowered chin bar still held by the right jaw pivot stud",
        )

    # Chin bar drops downward when opened.
    ctx.check(
        "open chin bar swings downward",
        chin_closed_aabb is not None
        and chin_open_aabb is not None
        and chin_open_aabb[0][2] < chin_closed_aabb[0][2] - 0.025,
        details=(
            f"closed bottom z={chin_closed_aabb[0][2] if chin_closed_aabb else None}, "
            f"open bottom z={chin_open_aabb[0][2] if chin_open_aabb else None}"
        ),
    )
    # The lower guard moves away from the eye-port/crown region instead of
    # rising into it.
    ctx.check(
        "open chin bar stays below the closed top edge",
        chin_closed_aabb is not None
        and chin_open_aabb is not None
        and chin_open_aabb[1][2] < chin_closed_aabb[1][2] + 0.010,
        details=(
            f"closed top z={chin_closed_aabb[1][2] if chin_closed_aabb else None}, "
            f"open top z={chin_open_aabb[1][2] if chin_open_aabb else None}"
        ),
    )

    # Chin bar has visible hinge arms integrated into the shell (the arm end
    # caps at the pivot points read as visible hinge knuckles).
    chin_bar_visuals = [v.name for v in chin_bar.visuals]
    ctx.check(
        "chin bar has visible hinge shell with arms",
        "chin_bar_shell" in chin_bar_visuals,
        details=f"visuals={chin_bar_visuals}",
    )

    # Model has at least two non-fixed articulations (visor + chin bar).
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least two non-fixed articulations (visor + chin bar)",
        len(non_fixed) >= 2,
        details=f"got {len(non_fixed)} non-fixed articulations",
    )

    return ctx.report()


object_model = build_object_model()
