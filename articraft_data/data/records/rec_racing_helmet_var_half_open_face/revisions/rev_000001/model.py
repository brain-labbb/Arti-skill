from __future__ import annotations

# Open-face racing helmet with a flip-up clear visor.
#
# Variant of the full-face racing helmet: the lower chin bar has been removed
# and the face opening enlarged to create a half / open-face racing helmet.
# The shell still covers the top, sides, and back of the head, but the front
# is open from the brow line down to the neck rim. The visor still pivots on
# two side studs near the temples and flips up-and-back over the crown via a
# single revolute joint about the Y axis.
#
# Build strategy:
# - Shell: a thick-walled rounded dome (CadQuery ellipsoid shelled and trimmed)
#   with a large face opening (no chin bar). Glossy red with a black face rim.
# - Face rim: thin black padding strip around the face opening edge, replacing
#   the chin trim of the full-face variant.
# - Visor: a tall thin curved clear panel covering the large face opening,
#   with a black edge trim band. Separate part carried by the pivots.
# - Pivot studs: two black hinge pins, one per temple, mounted on the shell.
# - Visor side arms: flat black capsule plates linking visor band to pivots.

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
# Geometry constants (meters). +X forward (face), +Z up.
# HEAD_CZ lifts everything so the neck rim rests on the ground plane.
# ---------------------------------------------------------------------------
HEAD_RX = 0.125
HEAD_RY = 0.100
HEAD_RZ = 0.110
SHELL_WALL = 0.012

NECK_Z = -0.085
HEAD_CZ = -NECK_Z

# Pivot location: near the temples, at the top edge of the face opening.
PIVOT_X = 0.055
PIVOT_Z = 0.055

# Face opening: the entire front wall below the brow line is removed (no chin
# bar). The opening extends from the brow down to the neck rim at the front.
FACE_OPEN_Z_HI = 0.058   # top of face opening (brow line)

# Face opening cut box: punches through the front wall only. Y extent is
# narrower than the shell's widest point so the side walls remain intact.
FACE_CUT_X_LO = 0.02
FACE_CUT_X_HI = 0.20
FACE_CUT_Y_HALF = 0.078
FACE_CUT_FILLET = 0.025

# Visor side-edge relief: removes visor material near the pivot that would
# sweep into the shell side walls during flip-up rotation.
VISOR_SIDE_CUT_X = 0.08
VISOR_SIDE_CUT_Z = 0.035

# Visor side arms
ARM_ANCHOR_X = 0.085
ARM_ANCHOR_Z = 0.025
ARM_R = 0.013
ARM_Y_IN = 0.084
ARM_T = 0.008

# Hinge pin (stud)
STUD_RADIUS = 0.011
STUD_LEN = 0.035
STUD_Y_CENTER = 0.0775


def _full_ellipsoid(rx: float, ry: float, rz: float) -> cq.Solid:
    """Full ellipsoid centered at the origin."""
    sphere = cq.Solid.makeSphere(1.0, angleDegrees1=-90, angleDegrees2=90)
    matrix = cq.Matrix([
        [rx, 0.0, 0.0, 0.0],
        [0.0, ry, 0.0, 0.0],
        [0.0, 0.0, rz, 0.0],
    ])
    return sphere.transformGeometry(matrix)


def _build_shell() -> cq.Workplane:
    """Open-face helmet shell: thick-walled ellipsoid with neck trim and a
    large face opening (no chin bar). Side walls and back remain intact."""
    outer = _full_ellipsoid(HEAD_RX, HEAD_RY, HEAD_RZ)
    inner = _full_ellipsoid(
        HEAD_RX - SHELL_WALL, HEAD_RY - SHELL_WALL, HEAD_RZ - SHELL_WALL
    )
    shell = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))

    # Trim below neck plane: flat resting rim plus open neck hole.
    neck_cut = (
        cq.Workplane("XY").box(0.5, 0.5, 0.3).translate((0.0, 0.0, NECK_Z - 0.15))
    )
    shell = shell.cut(neck_cut)

    # Large face opening: removes the entire front wall below the brow line.
    # The cut extends from well below the neck plane up to the brow line.
    # The Y extent is narrower than the shell width so the side walls are
    # preserved, creating the open-face helmet's characteristic shape.
    face_z_lo = NECK_Z - 0.02
    face_z_center = (face_z_lo + FACE_OPEN_Z_HI) / 2.0
    face_z_h = FACE_OPEN_Z_HI - face_z_lo
    face_x_center = (FACE_CUT_X_LO + FACE_CUT_X_HI) / 2.0
    face_cut = (
        cq.Workplane("XY")
        .box(FACE_CUT_X_HI - FACE_CUT_X_LO, 2.0 * FACE_CUT_Y_HALF, face_z_h)
        .edges("|Z")
        .fillet(FACE_CUT_FILLET)
        .translate((face_x_center, 0.0, face_z_center))
    )
    shell = shell.cut(face_cut)

    return shell


def _build_face_rim() -> cq.Workplane:
    """Black rubber padding around the face opening edge: a thin frame that
    follows the face opening perimeter on the shell surface, replacing the
    chin trim of the full-face variant."""
    rim_t = 0.005
    rim_w = 0.008

    # Thin ellipsoidal shell slightly proud of the helmet surface
    rim_outer = _full_ellipsoid(
        HEAD_RX + rim_t, HEAD_RY + rim_t, HEAD_RZ + rim_t
    )
    rim_inner = _full_ellipsoid(
        HEAD_RX - 0.002, HEAD_RY - 0.002, HEAD_RZ - 0.002
    )
    rim_shell = cq.Workplane(obj=rim_outer).cut(cq.Workplane(obj=rim_inner))

    # Frame: region between an outer box (face cut + rim_w) and an inner box
    # (face cut - rim_w), isolating the rim to the face opening perimeter.
    face_z_lo = NECK_Z - 0.02
    of_xc = (FACE_CUT_X_LO + FACE_CUT_X_HI) / 2.0
    of_zc = (face_z_lo + FACE_OPEN_Z_HI) / 2.0

    outer_box = (
        cq.Workplane("XY")
        .box(
            FACE_CUT_X_HI - FACE_CUT_X_LO + 2 * rim_w,
            2.0 * (FACE_CUT_Y_HALF + rim_w),
            FACE_OPEN_Z_HI - face_z_lo + 2 * rim_w,
        )
        .edges("|Z")
        .fillet(FACE_CUT_FILLET + rim_w)
        .translate((of_xc, 0.0, of_zc))
    )

    inner_box = (
        cq.Workplane("XY")
        .box(
            FACE_CUT_X_HI - FACE_CUT_X_LO - 2 * rim_w,
            2.0 * max(0.01, FACE_CUT_Y_HALF - rim_w),
            FACE_OPEN_Z_HI - face_z_lo - 2 * rim_w,
        )
        .edges("|Z")
        .fillet(max(0.005, FACE_CUT_FILLET - rim_w))
        .translate((of_xc, 0.0, of_zc))
    )

    frame = outer_box.cut(inner_box)
    return rim_shell.intersect(frame)


def _build_visor() -> cq.Workplane:
    """Tall curved clear visor for the open-face helmet. Covers the large face
    opening from the brow down to the mid-lower face."""
    rx = HEAD_RX + 0.014
    ry = HEAD_RY + 0.014
    rz = HEAD_RZ + 0.014
    wall = 0.004

    outer = _full_ellipsoid(rx, ry, rz)
    inner = _full_ellipsoid(rx - wall, ry - wall, rz - wall)
    visor_shell = cq.Workplane(obj=outer).cut(cq.Workplane(obj=inner))

    # Keep the front strip covering the face opening with margin.
    # The visor extends well below the brow to cover the open face.
    visor_z_lo = -0.048
    visor_z_hi = FACE_OPEN_Z_HI + 0.010
    z_center = (visor_z_lo + visor_z_hi) / 2.0
    z_h = visor_z_hi - visor_z_lo
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
    """Remove side-wrap material near the pivot so the raised visor clears the
    shell side walls."""
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
    z_center = FACE_OPEN_Z_HI + 0.001
    keep = (
        cq.Workplane("XY")
        .box(0.22, 0.26, 0.012)
        .translate((0.16, 0.0, z_center))
    )
    return _cut_visor_side_relief(band.intersect(keep))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_helmet_open_face")

    red_gloss = model.material("shell_red", rgba=(0.78, 0.05, 0.06, 1.0))
    trim_black = model.material("trim_black", rgba=(0.06, 0.06, 0.07, 1.0))
    visor_clear = model.material("visor_clear", rgba=(0.12, 0.13, 0.16, 0.45))
    stud_black = model.material("stud_black", rgba=(0.10, 0.10, 0.11, 1.0))

    # ----- Shell (root) - open-face, no chin bar -----
    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_build_shell(), "shell"),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=red_gloss,
        name="shell_dome",
    )
    shell.visual(
        mesh_from_cadquery(_build_face_rim(), "face_rim"),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=trim_black,
        name="face_rim",
    )
    shell.inertial = Inertial.from_geometry(
        Cylinder(radius=HEAD_RY, length=2.0 * HEAD_RZ),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
    )

    # ----- Pivot studs (mounted on shell, define the hinge line) -----
    for side, sy in (("left", 1.0), ("right", -1.0)):
        stud = model.part(f"pivot_stud_{side}")
        stud.visual(
            Cylinder(radius=STUD_RADIUS, length=STUD_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stud_black,
            name=f"pivot_stud_{side}",
        )
        stud.inertial = Inertial.from_geometry(
            Cylinder(radius=STUD_RADIUS, length=STUD_LEN), mass=0.02
        )
        model.articulation(
            f"shell_to_pivot_{side}",
            ArticulationType.FIXED,
            parent=shell,
            child=stud,
            origin=Origin(xyz=(PIVOT_X, sy * STUD_Y_CENTER, PIVOT_Z + HEAD_CZ)),
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
        Cylinder(radius=0.10, length=0.12),
        mass=0.14,
        origin=Origin(xyz=(-PIVOT_X + 0.10, 0.0, -PIVOT_Z + 0.02)),
    )

    # Revolute hinge: axis along -Y so positive q flips the visor up and back
    # over the crown. The temple pivot sits near the top of the face opening;
    # the closed visor hangs forward and below it.
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

    # Pivot studs intentionally embed their inner end into the shell wall.
    ctx.allow_overlap(
        stud_l, shell,
        reason="Left pivot stud seated into shell wall as hinge boss.",
    )
    ctx.allow_overlap(
        stud_r, shell,
        reason="Right pivot stud seated into shell wall as hinge boss.",
    )
    # Visor side arms ride on the temple studs (hinge pin capture).
    ctx.allow_overlap(
        stud_l, visor,
        reason="Left visor arm pinned on temple stud it rotates about.",
    )
    ctx.allow_overlap(
        stud_r, visor,
        reason="Right visor arm pinned on temple stud it rotates about.",
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

    # --- Closed visor covers the face opening ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_origin_gap(
            visor, shell, axis="x", min_gap=0.05,
            name="closed visor sits in front of shell center",
        )
        ctx.expect_overlap(
            visor, shell, axes="z", min_overlap=0.04,
            name="closed visor covers face opening height",
        )
        ctx.expect_contact(
            visor, stud_l, name="closed visor arm rides the left pivot stud",
        )
        ctx.expect_contact(
            visor, stud_r, name="closed visor arm rides the right pivot stud",
        )
        visor_closed_aabb = ctx.part_world_aabb(visor)

    # --- Pivot studs are anchored to the shell ---
    ctx.expect_contact(stud_l, shell, name="left pivot stud anchored to shell")
    ctx.expect_contact(stud_r, shell, name="right pivot stud anchored to shell")
    ctx.expect_origin_distance(
        stud_l, stud_r, axes="y", min_dist=0.15,
        name="pivot studs straddle the head",
    )

    # --- Open-face: visor extends well below the brow, proving the face
    # opening is large (no chin bar to limit it) ---
    ctx.check(
        "open-face visor extends below brow line",
        visor_closed_aabb is not None
        and visor_closed_aabb[0][2] < PIVOT_Z + HEAD_CZ - 0.04,
        details=(
            f"visor bottom z={visor_closed_aabb[0][2] if visor_closed_aabb else None}, "
            f"pivot z={PIVOT_Z + HEAD_CZ}"
        ),
    )

    # --- Open-face: shell has no chin bar (front-bottom is open) ---
    # Verify indirectly: the visor panel's height (z extent) is significantly
    # taller than a full-face visor, confirming the large face opening it covers.
    if visor_closed_aabb is not None:
        visor_z_extent = visor_closed_aabb[1][2] - visor_closed_aabb[0][2]
        ctx.check(
            "visor panel is tall enough for open-face coverage",
            visor_z_extent > 0.09,
            details=f"visor z extent = {visor_z_extent:.4f}m (expected > 0.09m)",
        )

    # --- Raising the joint flips the visor up and clears it off the face ---
    with ctx.pose({hinge: math.radians(95.0)}):
        visor_open_aabb = ctx.part_world_aabb(visor)
        ctx.expect_contact(
            visor, stud_l, name="raised visor still held by the left pivot stud",
        )
        ctx.expect_contact(
            visor, stud_r, name="raised visor still held by the right pivot stud",
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
    # For the open-face variant the tall visor's bottom swings forward when
    # flipped up (it is far below the pivot). Instead of retraction, verify
    # the raised visor's top clears above the shell crown.
    shell_aabb = ctx.part_world_aabb(shell)
    ctx.check(
        "raised visor top clears above shell crown",
        visor_open_aabb is not None
        and shell_aabb is not None
        and visor_open_aabb[1][2] > shell_aabb[1][2] + 0.010,
        details=(
            f"open visor top z={visor_open_aabb[1][2] if visor_open_aabb else None}, "
            f"shell crown z={shell_aabb[1][2] if shell_aabb else None}"
        ),
    )
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

    return ctx.report()


object_model = build_object_model()
