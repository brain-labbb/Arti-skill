from __future__ import annotations

# Racing helmet with a flip-up clear visor.
#
# Real object (from reference image): a glossy red full-face karting / racing
# helmet (Ferrari livery) with a black lower trim and a clear, lightly tinted
# visor. The visor is the primary mechanism: it pivots on two side studs near
# the temples and flips up-and-back over the crown of the shell. That motion is
# a single revolute joint about the left-right (Y) axis through the two pivots.
#
# Build strategy:
# - Shell: a thick-walled rounded dome (CadQuery sphere/ellipsoid shelled and
#   trimmed) with a front eye-port opening cut into it. Glossy red.
# - Chin bar / lower trim: a black rounded band fused around the lower front so
#   the silhouette reads as a real helmet, not a bare dome.
# - Visor: a thin curved clear panel that wraps the front eye port, with a
#   black edge trim band. It is a separate part carried by the pivots.
# - Pivot studs: two black hinge pins, one per temple, mounted on the shell,
#   that establish the visor hinge line.
# - Visor side arms: flat black capsule plates, one per side of the visor,
#   running from the visor band edge to the pivot and riding on the stud, so
#   the visor is mechanically connected to its hinge at every angle.

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

# Pivot location: near the temples, at the top edge of the eye port (as in the
# photo, the visor studs sit level with the visor's upper edge).
PIVOT_X = 0.055
PIVOT_Z = 0.055

# Eye port opening (front of shell), measured from head center. The shell
# below PORT_Z_LO at the front is the chin bar.
PORT_Z_LO = -0.012
PORT_Z_HI = 0.055

# Eye-port cut box: punches through the front wall only. Its inner end (X_LO)
# stops inside the helmet cavity, well short of the rear interior wall, so the
# port is a window, not a through-channel.
PORT_CUT_X_LO = 0.04
PORT_CUT_X_HI = 0.18
PORT_CUT_Y_HALF = 0.075

# Visor side-edge relief: when the visor flips up about the temple pivot, any
# side-wrap material above the pivot would sweep back into the shell's side
# walls. Real visors slope their side edge forward near the pivot for exactly
# this reason; we remove visor material with x < SIDE_CUT_X above SIDE_CUT_Z
# (this only affects the side wrap -- at the front the band is well beyond
# SIDE_CUT_X for those heights).
VISOR_SIDE_CUT_X = 0.08
VISOR_SIDE_CUT_Z = 0.035

# Visor side arms: the relief cut above removes all visor band material near
# the pivot (x < 0.08 above z = 0.035), so without arms the visor would float
# next to its own hinge. Each arm is a flat capsule plate in the XZ plane that
# runs from the pivot to an anchor point inside the visor band (the band wall
# at the arm's Y range spans x ~0.068..0.090 around z = 0.025, so the anchor
# cap embeds into it). The plate's inner face (ARM_Y_IN) sits just outside the
# shell's widest bulge under the swept arm footprint (~0.083), so the arm
# clears the shell side wall at every hinge angle (rotation about Y preserves
# each point's Y).
ARM_ANCHOR_X = 0.085
ARM_ANCHOR_Z = 0.025
ARM_R = 0.013
ARM_Y_IN = 0.084
ARM_T = 0.008

# Hinge pin (stud) along Y: inner end seated in the shell wall, shaft passing
# through the visor arm plate, outer end capping it as a visible pivot boss.
STUD_RADIUS = 0.011
STUD_LEN = 0.035
STUD_Y_CENTER = 0.0775

# ---------------------------------------------------------------------------
# Rear occipital detail panel: fixed shell visuals on the back of the helmet.
# The change axis is no longer a front sun-peak; it is a refined rear shell
# surface treatment made from a dark inset panel plus loop-emitted raised vent
# slits and chevron ridges. These are inline shell visuals, not extra fixed
# parts, so the original visor articulation remains the only live mechanism.
# ---------------------------------------------------------------------------
REAR_PANEL_X_LO = -0.130
REAR_PANEL_X_HI = -0.072
REAR_PANEL_Y_HALF = 0.060
REAR_PANEL_Z_LO = -0.010
REAR_PANEL_Z_HI = 0.068
REAR_SLIT_COUNT = 6
REAR_SLIT_STEP_Z = 0.010
REAR_SLIT_BASE_Z = 0.002
REAR_SLIT_X_THICKNESS = 0.004
REAR_SLIT_Z_THICKNESS = 0.0022
REAR_RIDGE_RADIUS = 0.0028
REAR_RIDGE_THICKNESS = 0.004


def _full_ellipsoid(rx: float, ry: float, rz: float) -> cq.Solid:
    """Full (not hemispherical) ellipsoid centered at the origin.

    NOTE: cq.Solid.makeSphere defaults to angleDegrees1=0, which yields only
    the UPPER HEMISPHERE. angleDegrees1=-90 is required for a full sphere.
    """
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
    """Glossy red helmet shell: a thick-walled full ellipsoid (rounded crown,
    sides coming down over the jaw, chin bar across the front) trimmed flat at
    the neck so the cavity opens at the bottom, with a bounded eye-port window
    cut through the front wall only."""
    outer_ell = _full_ellipsoid(HEAD_RX, HEAD_RY, HEAD_RZ)
    inner_ell = _full_ellipsoid(
        HEAD_RX - SHELL_WALL, HEAD_RY - SHELL_WALL, HEAD_RZ - SHELL_WALL
    )
    shell = cq.Workplane(obj=outer_ell).cut(cq.Workplane(obj=inner_ell))

    # Trim away everything below the neck plane: flat resting rim plus an open
    # neck hole (the inner cavity reaches below NECK_Z, so the cut exposes it).
    neck_cut = (
        cq.Workplane("XY").box(0.5, 0.5, 0.3).translate((0.0, 0.0, NECK_Z - 0.15))
    )
    shell = shell.cut(neck_cut)

    # Carve the front eye-port window. The cut box spans only the front wall:
    # its rear face (PORT_CUT_X_LO) ends inside the cavity, so the rear interior
    # wall stays intact and the port is a window, not a see-through channel.
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

    return shell


def _build_chin_trim() -> cq.Workplane:
    """Black trim restricted to the chin-bar region: a thin overlay band on the
    front of the shell, below the eye port and above the neck rim (as in the
    photo, only the chin bar under the visor is black)."""
    band_outer = _full_ellipsoid(HEAD_RX + 0.004, HEAD_RY + 0.004, HEAD_RZ + 0.004)
    band_inner = _full_ellipsoid(
        HEAD_RX - SHELL_WALL - 0.002,
        HEAD_RY - SHELL_WALL - 0.002,
        HEAD_RZ - SHELL_WALL - 0.002,
    )
    band_wp = cq.Workplane(obj=band_outer).cut(cq.Workplane(obj=band_inner))

    # Keep only the chin-bar wedge: front of the helmet (x > 0.03), vertically
    # between just above the neck rim and the bottom of the eye port.
    z_lo = NECK_Z + 0.003
    z_hi = PORT_Z_LO
    keep = (
        cq.Workplane("XY")
        .box(0.20, 0.34, z_hi - z_lo)
        .translate((0.03 + 0.10, 0.0, (z_lo + z_hi) / 2.0))
    )
    band_wp = band_wp.intersect(keep)
    return band_wp


def _build_visor() -> cq.Workplane:
    """Thin curved clear visor panel that wraps the front eye port, authored in
    the shell frame (closed position). It rides at a slightly larger radius than
    the shell and chin trim so the closed visor clears them with a thin air
    gap."""
    rx = HEAD_RX + 0.014
    ry = HEAD_RY + 0.014
    rz = HEAD_RZ + 0.014
    wall = 0.004

    outer = _full_ellipsoid(rx, ry, rz)
    inner = _full_ellipsoid(rx - wall, ry - wall, rz - wall)
    visor_shell = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))

    # Keep only the front face strip covering the eye port plus a margin so the
    # closed visor fully covers the opening. The keep box starts at X=0.05 so
    # the visor wraps only the front of the shell and does not dip back into
    # the chin trim band. The skirt below the port bottom is kept to 1 mm:
    # the pose tests measure the part AABB corner-wise, and a deeper skirt's
    # (xmin, zmin) corner swings around to the front at full lift, reading as
    # the raised visor failing to retract over the crown.
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
    """Remove the side-wrap material above the pivot line (x < VISOR_SIDE_CUT_X
    and z > VISOR_SIDE_CUT_Z) so the raised visor clears the shell's side
    walls. See VISOR_SIDE_CUT_* notes."""
    relief = (
        cq.Workplane("XY")
        .box(0.30, 0.30, 0.20)
        .translate((VISOR_SIDE_CUT_X - 0.15, 0.0, VISOR_SIDE_CUT_Z + 0.10))
    )
    return panel.cut(relief)


def _build_visor_arm(side: float) -> cq.Workplane:
    """Flat capsule plate linking the visor band to the temple pivot on one
    side (+1 left / -1 right), authored in the head-centered frame. The pivot
    cap is concentric with the hinge axis, so it stays seated on the stud at
    every joint angle."""
    ux = ARM_ANCHOR_X - PIVOT_X
    uz = ARM_ANCHOR_Z - PIVOT_Z
    length = math.hypot(ux, uz)
    ux, uz = ux / length, uz / length
    px, pz = -uz * ARM_R, ux * ARM_R
    # Single closed capsule wire (two straight flanks + two semicircular end
    # caps), extruded once. Building the caps as separate cylinders and
    # unioning them would create tangent-face booleans, which OCCT grinds on.
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
    # The "XZ" workplane extrudes toward -Y, so the plate spans y in
    # [-ARM_T, 0]; shift it outboard per side.
    shift = (ARM_Y_IN + ARM_T) if side > 0 else -ARM_Y_IN
    return bar.translate((0.0, shift, 0.0))


def _build_visor_arms() -> cq.Workplane:
    return _build_visor_arm(1.0).union(_build_visor_arm(-1.0))


def _build_visor_trim() -> cq.Workplane:
    """Black trim band along the top edge of the visor: a snug sleeve hugging
    the visor band so its swept path stays just outside the visor's own."""
    rx = HEAD_RX + 0.016
    ry = HEAD_RY + 0.016
    rz = HEAD_RZ + 0.016
    wall = 0.0055
    outer = _full_ellipsoid(rx, ry, rz)
    inner = _full_ellipsoid(rx - wall, ry - wall, rz - wall)
    band = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))
    # Thin horizontal slab near the top edge of the visor, front-only. Kept
    # low (top at PORT_Z_HI + 0.007) so its swept path clears the crown.
    z_center = PORT_Z_HI + 0.001
    keep = (
        cq.Workplane("XY")
        .box(0.22, 0.26, 0.012)
        .translate((0.16, 0.0, z_center))
    )
    return _cut_visor_side_relief(band.intersect(keep))


def _rear_surface_x(y: float, z: float, outward: float = 0.005) -> float:
    """Approximate the rear shell surface x coordinate for a head-centered y/z.

    The detail slits are tiny raised visuals. Placing them just outside the
    ellipsoid keeps them visually seated on the back of the helmet instead of
    floating in a flat panel far behind it.
    """
    inside = 1.0 - (y / HEAD_RY) ** 2 - (z / HEAD_RZ) ** 2
    return -HEAD_RX * math.sqrt(max(inside, 0.02)) - outward


def _build_rear_occipital_panel() -> cq.Workplane:
    """Dark curved inset panel on the rear shell, cut from an ellipsoid sleeve."""
    outer = _full_ellipsoid(HEAD_RX + 0.004, HEAD_RY + 0.004, HEAD_RZ + 0.004)
    inner = _full_ellipsoid(
        HEAD_RX - SHELL_WALL - 0.001,
        HEAD_RY - SHELL_WALL - 0.001,
        HEAD_RZ - SHELL_WALL - 0.001,
    )
    sleeve = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))
    keep = (
        cq.Workplane("XY")
        .box(
            REAR_PANEL_X_HI - REAR_PANEL_X_LO,
            2.0 * REAR_PANEL_Y_HALF,
            REAR_PANEL_Z_HI - REAR_PANEL_Z_LO,
        )
        .edges("|Z")
        .fillet(0.018)
        .translate(
            (
                (REAR_PANEL_X_LO + REAR_PANEL_X_HI) / 2.0,
                0.0,
                (REAR_PANEL_Z_LO + REAR_PANEL_Z_HI) / 2.0,
            )
        )
    )
    return sleeve.intersect(keep)


def _build_rear_vent_slit(width: float) -> cq.Workplane:
    """One raised horizontal rear vent strip, centered at the origin."""
    return (
        cq.Workplane("XY")
        .box(REAR_SLIT_X_THICKNESS, width, REAR_SLIT_Z_THICKNESS)
        .edges("|X")
        .fillet(0.001)
    )


def _build_rear_chevron_ridge(side: float) -> cq.Workplane:
    """One diagonal raised ridge for the rear chevron texture."""
    y0, z0 = 0.0, 0.055
    y1, z1 = side * 0.038, 0.020
    uy = y1 - y0
    uz = z1 - z0
    length = math.hypot(uy, uz)
    uy, uz = uy / length, uz / length
    py, pz = -uz * REAR_RIDGE_RADIUS, uy * REAR_RIDGE_RADIUS
    ridge = (
        cq.Workplane("YZ")
        .moveTo(y0 + py, z0 + pz)
        .lineTo(y1 + py, z1 + pz)
        .threePointArc(
            (y1 + uy * REAR_RIDGE_RADIUS, z1 + uz * REAR_RIDGE_RADIUS),
            (y1 - py, z1 - pz),
        )
        .lineTo(y0 - py, z0 - pz)
        .threePointArc(
            (y0 - uy * REAR_RIDGE_RADIUS, z0 - uz * REAR_RIDGE_RADIUS),
            (y0 + py, z0 + pz),
        )
        .close()
        .extrude(REAR_RIDGE_THICKNESS)
    )
    return ridge.translate((_rear_surface_x(0.0, 0.036, 0.001), 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_helmet_flip_visor")

    red_gloss = model.material("shell_red", rgba=(0.78, 0.05, 0.06, 1.0))
    trim_black = model.material("trim_black", rgba=(0.06, 0.06, 0.07, 1.0))
    visor_clear = model.material("visor_clear", rgba=(0.12, 0.13, 0.16, 0.45))
    stud_black = model.material("stud_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rear_gunmetal = model.material("rear_gunmetal", rgba=(0.08, 0.085, 0.095, 1.0))

    # ----- Shell (root) -----
    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_build_shell(), "shell"),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=red_gloss,
        name="shell_dome",
    )
    shell.visual(
        mesh_from_cadquery(_build_chin_trim(), "chin_trim"),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=trim_black,
        name="chin_trim",
    )
    # Rear occipital detail: fixed curved panel plus loop-emitted raised slits
    # and chevron ridges. These are shell visuals, so no fixed-joint decoration
    # parts are introduced.
    shell.visual(
        mesh_from_cadquery(_build_rear_occipital_panel(), "rear_occipital_panel"),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=rear_gunmetal,
        name="rear_occipital_panel",
    )
    for i in range(REAR_SLIT_COUNT):
        z = REAR_SLIT_BASE_Z + i * REAR_SLIT_STEP_Z
        # Slightly taper the slits at top/bottom so the group follows the oval
        # occipital patch instead of reading as a flat barcode.
        taper = 1.0 - 0.08 * abs(i - (REAR_SLIT_COUNT - 1) / 2.0)
        width = 0.082 * taper
        shell.visual(
            mesh_from_cadquery(_build_rear_vent_slit(width), f"rear_vent_slit_{i}"),
            origin=Origin(xyz=(_rear_surface_x(0.0, z, 0.001), 0.0, z + HEAD_CZ)),
            material=rear_gunmetal,
            name=f"rear_vent_slit_{i}",
        )
    for i in range(2):
        side = 1.0 if i == 0 else -1.0
        shell.visual(
            mesh_from_cadquery(_build_rear_chevron_ridge(side), f"rear_chevron_ridge_{i}"),
            origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
            material=rear_gunmetal,
            name=f"rear_chevron_ridge_{i}",
        )
    shell.inertial = Inertial.from_geometry(
        Cylinder(radius=HEAD_RY, length=2.0 * HEAD_RZ),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
    )

    # ----- Pivot studs (mounted on shell, define the hinge line) -----
    for side, sy in (("left", 1.0), ("right", -1.0)):
        stud = model.part(f"pivot_stud_{side}")
        # Cylinder along local +Z; rotate so it points along the world Y axis
        # (the hinge line). pitch about... we rotate the cylinder so its long
        # axis lies along Y by rotating +90deg about X.
        stud.visual(
            Cylinder(radius=STUD_RADIUS, length=STUD_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stud_black,
            name=f"pivot_stud_{side}",
        )
        stud.inertial = Inertial.from_geometry(
            Cylinder(radius=STUD_RADIUS, length=STUD_LEN), mass=0.02
        )
        # Mount the pin at the temple. Its inner end reaches into the shell
        # wall (shell wall spans ~Y=0.059..0.075 here) so it is anchored, not
        # floating; the shaft passes through the visor side-arm plate
        # (y 0.084..0.092) and its outer end caps the arm as the pivot boss.
        model.articulation(
            f"shell_to_pivot_{side}",
            ArticulationType.FIXED,
            parent=shell,
            child=stud,
            origin=Origin(xyz=(PIVOT_X, sy * STUD_Y_CENTER, PIVOT_Z + HEAD_CZ)),
        )

    # ----- Visor (clear flip-up panel) -----
    visor = model.part("visor")
    # The visor mesh is authored in the head-centered frame (closed pose). The
    # joint origin is (PIVOT_X, 0, PIVOT_Z + HEAD_CZ); the visual origin
    # (-PIVOT_X, 0, -PIVOT_Z) cancels the pivot offset so that at q=0 the
    # panel lands in its closed pose, lifted by HEAD_CZ like the shell.
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

    # Revolute hinge: axis along Y (the temple-to-temple line). The visor
    # geometry, after the cancelling origin, sits forward (+X) and below/at the
    # pivot. Positive q about +Y rotates the front of the visor upward and back
    # over the crown (flip-up). Verified by the pose test.
    # The temple pivot sits near the top of the eye port; the closed visor hangs
    # forward and below it. Axis -Y, by the right-hand rule, swings the whole
    # panel up so its lower edge lifts off the face and the top rolls up over the
    # crown -- the real flip-up motion.
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    visor = object_model.get_part("visor")
    stud_l = object_model.get_part("pivot_stud_left")
    stud_r = object_model.get_part("pivot_stud_right")
    hinge = object_model.get_articulation("shell_to_visor")

    # The pivot studs intentionally embed their inner end into the shell wall so
    # they are anchored pivot bosses rather than floating cylinders.
    ctx.allow_overlap(
        stud_l,
        shell,
        reason="Left pivot stud inner end is seated into the shell wall as a hinge boss.",
    )
    ctx.allow_overlap(
        stud_r,
        shell,
        reason="Right pivot stud inner end is seated into the shell wall as a hinge boss.",
    )
    # The visor side arms ride on the temple studs (the hinge pin passes
    # through each arm plate). That interpenetration is the hinge capture.
    ctx.allow_overlap(
        stud_l,
        visor,
        reason="Left visor side arm is pinned on the temple stud it rotates about.",
    )
    ctx.allow_overlap(
        stud_r,
        visor,
        reason="Right visor side arm is pinned on the temple stud it rotates about.",
    )

    # --- Joint is the flip-up visor hinge, revolute about Y ---
    ctx.check(
        "visor hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {hinge.articulation_type}",
    )
    ax = hinge.axis
    ctx.check(
        "visor hinge axis is left-right (Y)",
        abs(ax[1]) > 0.9 and abs(ax[0]) < 0.1 and abs(ax[2]) < 0.1,
        details=f"axis={ax}",
    )

    # --- Closed visor covers the front eye port and sits in front of shell ---
    with ctx.pose({hinge: 0.0}):
        # Visor front face is ahead of the shell center in +X.
        ctx.expect_origin_gap(
            visor,
            shell,
            axis="x",
            min_gap=0.05,
            name="closed visor sits in front of shell center",
        )
        # Visor footprint overlaps the shell front opening in the vertical span.
        ctx.expect_overlap(
            visor,
            shell,
            axes="z",
            min_overlap=0.04,
            name="closed visor covers eye-port height",
        )
        # The visor side arms must reach their hinge pins: the closed visor is
        # mechanically held on both studs, not floating next to them.
        ctx.expect_contact(
            visor, stud_l, name="closed visor arm rides the left pivot stud"
        )
        ctx.expect_contact(
            visor, stud_r, name="closed visor arm rides the right pivot stud"
        )
        visor_closed_aabb = ctx.part_world_aabb(visor)

    # --- Pivot studs are anchored to the shell (in contact, not floating) ---
    ctx.expect_contact(
        stud_l, shell, name="left pivot stud anchored to shell"
    )
    ctx.expect_contact(
        stud_r, shell, name="right pivot stud anchored to shell"
    )
    # Studs straddle the centerline on opposite sides.
    ctx.expect_origin_distance(
        stud_l, stud_r, axes="y", min_dist=0.15,
        name="pivot studs straddle the head",
    )

    # --- Raising the joint flips the visor up and clears it off the face ---
    with ctx.pose({hinge: math.radians(95.0)}):
        visor_open_aabb = ctx.part_world_aabb(visor)
        # The arm pivot caps are concentric with the hinge axis, so the raised
        # visor must still be seated on both studs (no floating at any angle).
        ctx.expect_contact(
            visor, stud_l, name="raised visor still held by the left pivot stud"
        )
        ctx.expect_contact(
            visor, stud_r, name="raised visor still held by the right pivot stud"
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
    # Flipping up rolls the visor back over the crown: its forward-most reach
    # retracts toward the head.
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
    # When flipped up, the visor's lower edge rises off the lower face: its
    # minimum Z lifts well above the closed eye-port bottom.
    ctx.check(
        "raised visor lower edge clears the face",
        visor_closed_aabb is not None
        and visor_open_aabb is not None
        and visor_open_aabb[0][2] > visor_closed_aabb[0][2] + 0.025,
        details=(
            f"closed bottom z={visor_closed_aabb[0][2] if visor_closed_aabb else None}, "
            f"open bottom z={visor_open_aabb[0][2] if visor_open_aabb else None}"
        ),
    )

    # --- Rear occipital detail mesh / texture ---
    panel_aabb = ctx.part_element_world_aabb(shell, elem="rear_occipital_panel")
    dome_aabb = ctx.part_element_world_aabb(shell, elem="shell_dome")
    ctx.check(
        "rear occipital panel exists on back of shell",
        panel_aabb is not None
        and dome_aabb is not None
        and panel_aabb[0][0] < -0.11
        and panel_aabb[1][0] < 0.0,
        details=f"panel_aabb={panel_aabb}, dome_aabb={dome_aabb}",
    )

    slit_aabbs = [
        ctx.part_element_world_aabb(shell, elem=f"rear_vent_slit_{i}")
        for i in range(REAR_SLIT_COUNT)
    ]
    ctx.check(
        "rear vent slits emitted by loop",
        all(aabb is not None for aabb in slit_aabbs),
        details=f"slit_aabbs={slit_aabbs}",
    )
    if all(aabb is not None for aabb in slit_aabbs):
        slit_zs = [
            (aabb[0][2] + aabb[1][2]) / 2.0
            for aabb in slit_aabbs
        ]
        ctx.check(
            "rear vent slits form a vertical texture stack",
            max(slit_zs) - min(slit_zs) > 0.045,
            details=f"slit_zs={slit_zs}",
        )
        ctx.check(
            "rear vent slits stay behind helmet center",
            all(aabb[1][0] < -0.08 for aabb in slit_aabbs),
            details=f"slit_aabbs={slit_aabbs}",
        )

    ridge_0 = ctx.part_element_world_aabb(shell, elem="rear_chevron_ridge_0")
    ridge_1 = ctx.part_element_world_aabb(shell, elem="rear_chevron_ridge_1")
    ctx.check(
        "rear chevron ridges exist",
        ridge_0 is not None and ridge_1 is not None,
        details=f"ridge_0={ridge_0}, ridge_1={ridge_1}",
    )
    if ridge_0 is not None and ridge_1 is not None:
        ridge_0_cy = (ridge_0[0][1] + ridge_0[1][1]) / 2.0
        ridge_1_cy = (ridge_1[0][1] + ridge_1[1][1]) / 2.0
        ctx.check(
            "rear chevron ridges are mirrored across centerline",
            ridge_0_cy * ridge_1_cy < 0.0,
            details=f"ridge_0_center_y={ridge_0_cy:.4f}, ridge_1_center_y={ridge_1_cy:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
