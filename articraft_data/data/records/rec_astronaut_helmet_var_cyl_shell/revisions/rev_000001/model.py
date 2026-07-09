from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# Costume-style NASA astronaut helmet (diving-bell variant).
# White hollow cylindrical barrel shell with domed top and a circular front
# opening, stepped ribbed neck-ring base, rear ribbed hose fitting, black
# interior chin lip, red NASA worm-logo decal and US flag decal, plus a
# flip-up amber bubble visor carried by a white frame that pivots on round
# side disks.

# Global layout (meters). The shell local frame is authored around its own
# origin; the shell visual and the visor articulation both sit at CENTER_Z
# above the ground so the neck ring rests on z = 0.
R_OUT = 0.150  # shell outer radius (cylinder radius)
R_IN = 0.142  # shell inner radius (hollow)
CENTER_Z = 0.160  # shell local-frame origin height above ground
OPENING_R = 0.092  # circular face-opening radius (cut along +X)
OPENING_ZC = 0.010  # opening/visor axis height above the local origin
DOME_START_Z = 0.080  # cylinder wall transitions to hemisphere dome here
NECK_CUT_Z = -0.105  # cylinder bottom (neck ring takes over below)
VISOR_R_IN = 0.152  # amber bubble inner radius (2 mm clear of shell)
VISOR_R_OUT = 0.157
FRAME_R_IN = 0.154  # raised white frame sits proud of the bubble
FRAME_R_OUT = 0.161
PIVOT_Y = 0.1498  # inner face of each pivot disk: seated on the shell with a
# 0.2 mm embed so the hinge reads as mounted (tiny cap, below volume tol)

# NOTE: booleans are done on cq Shapes (.val()) rather than Workplanes; the
# Workplane-level intersect() produced corrupt results for these sphere/cyl
# combinations in this CadQuery build.


def _sphere(radius: float) -> cq.Shape:
    return cq.Workplane("XY").sphere(radius).val()


def _sphere_shell(r_out: float, r_in: float) -> cq.Shape:
    return _sphere(r_out).cut(_sphere(r_in))


def _x_cylinder(radius: float, x0: float, x1: float, zc: float = 0.0) -> cq.Shape:
    """Solid cylinder along +X from x0 to x1, with its axis at z=zc."""
    return cq.Solid.makeCylinder(
        radius, x1 - x0, cq.Vector(x0, 0.0, zc), cq.Vector(1.0, 0.0, 0.0)
    )


def _y_cylinder(radius: float, y0: float, y1: float) -> cq.Shape:
    """Solid cylinder along the Y axis from y0 to y1, centered on the Y axis."""
    lo, hi = min(y0, y1), max(y0, y1)
    return cq.Solid.makeCylinder(
        radius, hi - lo, cq.Vector(0.0, lo, 0.0), cq.Vector(0.0, 1.0, 0.0)
    )


def _z_cylinder(radius: float, z0: float, z1: float) -> cq.Shape:
    lo, hi = min(z0, z1), max(z0, z1)
    return cq.Solid.makeCylinder(
        radius, hi - lo, cq.Vector(0.0, 0.0, lo), cq.Vector(0.0, 0.0, 1.0)
    )


def _box(sx: float, sy: float, sz: float, center: tuple[float, float, float]) -> cq.Shape:
    return cq.Workplane("XY").box(sx, sy, sz).translate(center).val()


def _hollow_hemisphere(r: float, z_center: float) -> cq.Shape:
    """Top hemisphere of a sphere of radius r, centered at z=z_center."""
    dome = _sphere(r).translate(cq.Vector(0.0, 0.0, z_center))
    # Remove the bottom half
    return dome.cut(_box(0.5, 0.5, 0.5, (0.0, 0.0, z_center - 0.25)))


def _build_shell_shape() -> cq.Shape:
    """White helmet shell: hollow cylindrical barrel with domed top (diving-bell
    style), circular face opening, stepped ribbed collar, rear ribbed hose
    fitting."""

    # --- Outer shell: vertical cylinder + hemisphere dome ---
    outer_cyl = _z_cylinder(R_OUT, NECK_CUT_Z, DOME_START_Z)
    outer_dome = _hollow_hemisphere(R_OUT, DOME_START_Z)
    outer = outer_cyl.fuse(outer_dome)

    # --- Inner cavity: slightly smaller cylinder + dome (hollow wall) ---
    inner_cyl = _z_cylinder(R_IN, NECK_CUT_Z - 0.001, DOME_START_Z)
    inner_dome = _hollow_hemisphere(R_IN, DOME_START_Z)
    inner = inner_cyl.fuse(inner_dome)

    # Hollow shell = outer minus inner
    shell = outer.cut(inner)

    # Shoulder plate: annular ring connecting the cylinder bottom rim to the
    # collar top, so the collar reads as mounted on the barrel.
    shoulder = _z_cylinder(R_OUT, NECK_CUT_Z - 0.004, NECK_CUT_Z + 0.004)
    shoulder = shoulder.cut(_z_cylinder(0.110, NECK_CUT_Z - 0.004, NECK_CUT_Z + 0.004))
    shell = shell.fuse(shoulder)

    # Circular face opening for the visor, cut along +X through the cylinder
    # wall. The axis sits a bit above the local origin so the opening clears
    # the neck-ring collar.
    shell = shell.cut(_x_cylinder(OPENING_R, 0.02, 0.25, OPENING_ZC))

    # Stepped neck-ring collar (upper tier + wider bottom tier), hollow inside.
    # The upper tier runs up to z=-0.100 so it genuinely fuses with the
    # cylinder wall.
    collar = _z_cylinder(0.112, -0.160, -0.100)
    collar = collar.fuse(_z_cylinder(0.118, -0.160, -0.140))
    collar = collar.cut(_z_cylinder(0.095, -0.165, -0.095))

    # Vertical rib notches around the upper collar rim (as in the reference).
    n_notches = 14
    for i in range(n_notches):
        ang = 2.0 * math.pi * i / n_notches
        collar = collar.cut(
            _box(0.016, 0.016, 0.030, (0.114, 0.0, -0.118)).rotate(
                cq.Vector(0, 0, 0), cq.Vector(0, 0, 1.0), math.degrees(ang)
            )
        )
    shell = shell.fuse(collar)

    # Rear ribbed hose fitting: radial stub with raised ring ribs, embedded
    # into the shell wall so it reads as mounted.
    stub = _x_cylinder(0.017, -0.176, -0.136)
    for i in range(3):
        x_rib = -0.172 + 0.012 * i
        stub = stub.fuse(_x_cylinder(0.023, x_rib, x_rib + 0.007))
    stub = stub.translate(cq.Vector(0.0, 0.0, -0.050))
    shell = shell.fuse(stub)

    return shell


def _build_interior_lip_shape() -> cq.Shape:
    """Black interior chin lip visible along the bottom of the face opening.
    A thin cylindrical shell band just inside the barrel wall."""
    # Cylindrical shell band between R=0.1405 and R=0.147, spanning the
    # opening height.
    lip = _z_cylinder(0.147, -0.085, 0.065).cut(_z_cylinder(0.1405, -0.085, 0.065))
    # Chin-height band, then slightly wider than the opening so the rim tucks
    # behind the shell wall (keeps the lip connected to the shell part).
    lip = lip.intersect(_box(0.4, 0.4, 0.045, (0.0, 0.0, -0.0575)))
    return lip.intersect(_x_cylinder(0.096, 0.01, 0.20, OPENING_ZC))


def _build_visor_bubble_shape() -> cq.Shape:
    """Amber transparent visor panel: cylindrical shell following the barrel
    curvature, protruding slightly proud of the barrel over the face opening."""
    # Cylindrical shell along Z, 5–12 mm proud of the barrel wall.
    outer = _z_cylinder(R_OUT + 0.012, OPENING_ZC - OPENING_R - 0.008,
                        OPENING_ZC + OPENING_R + 0.008)
    inner = _z_cylinder(R_OUT + 0.005, OPENING_ZC - OPENING_R - 0.008,
                        OPENING_ZC + OPENING_R + 0.008)
    bubble = outer.cut(inner)
    # Clip to the opening footprint along +X.
    return bubble.intersect(_x_cylinder(OPENING_R + 0.002, 0.0, 0.20, OPENING_ZC))


def _build_visor_frame_shape() -> cq.Shape:
    """White raised frame: bezel ring around the opening plus two side arms
    that run back along the barrel to the round pivot disks."""
    # Bezel ring: cylindrical shell band slightly wider than the opening.
    bezel_outer = _z_cylinder(R_OUT + 0.016, OPENING_ZC - OPENING_R - 0.014,
                              OPENING_ZC + OPENING_R + 0.014)
    bezel_inner = _z_cylinder(R_OUT + 0.004, OPENING_ZC - OPENING_R - 0.006,
                             OPENING_ZC + OPENING_R + 0.006)
    bezel = bezel_outer.cut(bezel_inner)
    bezel = bezel.intersect(
        _x_cylinder(OPENING_R + 0.016, 0.0, 0.20, OPENING_ZC)
    )
    # Cut out the inner opening so the bezel is a ring
    bezel = bezel.cut(_x_cylinder(OPENING_R - 0.004, -0.01, 0.21, OPENING_ZC))
    frame = bezel

    for side in (1.0, -1.0):
        # Curved arm strip hugging the cylinder from the bezel to the pivot.
        arm_outer = _z_cylinder(R_OUT + 0.011, -0.014, 0.014)
        arm_inner = _z_cylinder(R_OUT + 0.003, -0.014, 0.014)
        arm = arm_outer.cut(arm_inner)
        arm = arm.intersect(
            _box(0.20, 0.090, 0.028, (0.070, side * 0.125, 0.0))
        )
        frame = frame.fuse(arm)

        # Round pivot disk stack at the end of the arm.
        frame = frame.fuse(_y_cylinder(0.036, side * PIVOT_Y, side * (PIVOT_Y + 0.013)))
        frame = frame.fuse(
            _y_cylinder(0.026, side * (PIVOT_Y + 0.013), side * (PIVOT_Y + 0.019))
        )
    return frame


def _build_pivot_knob_shape(side: float) -> cq.Shape:
    """Gray central retaining knob with a small cross-bar on one pivot disk."""
    knob = _y_cylinder(0.015, side * (PIVOT_Y + 0.017), side * (PIVOT_Y + 0.026))
    bar = _box(0.020, 0.005, 0.006, (0.0, side * (PIVOT_Y + 0.0245), 0.0))
    return knob.fuse(bar)


def _cylinder_decal_origin(az: float, z_local: float, r: float) -> Origin:
    """Origin for a thin decal box tangent to the cylinder barrel at azimuth az
    and local height z_local.

    The decal Box is sized (thickness, width, height): local +X is the
    outward surface normal (horizontal, pointing away from the Z axis).
    """
    x = r * math.cos(az)
    y = r * math.sin(az)
    z = z_local + CENTER_Z
    return Origin(xyz=(x, y, z), rpy=(0.0, 0.0, az))


def _cylinder_decal_axes(az: float):
    """Local width (y) and height (z) axes of a decal tangent to the cylinder."""
    y_axis = (-math.sin(az), math.cos(az), 0.0)
    z_axis = (0.0, 0.0, 1.0)
    return y_axis, z_axis


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nasa_astronaut_costume_helmet")

    white = model.material("helmet_white", rgba=(0.93, 0.93, 0.90, 1.0))
    frame_white = model.material("frame_white", rgba=(0.96, 0.96, 0.94, 1.0))
    amber = model.material("visor_amber", rgba=(0.85, 0.55, 0.12, 0.55))
    black = model.material("interior_black", rgba=(0.07, 0.07, 0.07, 1.0))
    gray = model.material("hub_gray", rgba=(0.72, 0.73, 0.74, 1.0))
    red = model.material("logo_red", rgba=(0.86, 0.10, 0.08, 1.0))
    flag_blue = model.material("flag_blue", rgba=(0.13, 0.18, 0.45, 1.0))
    flag_stripes = model.material("flag_stripes", rgba=(0.80, 0.15, 0.15, 1.0))

    # --- Shell (root part) -------------------------------------------------
    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_build_shell_shape(), "shell_body"),
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        material=white,
        name="shell_body",
    )
    shell.visual(
        mesh_from_cadquery(_build_interior_lip_shape(), "interior_lip"),
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        material=black,
        name="interior_lip",
    )

    # Red NASA worm-logo decal on the jaw, below the visor opening: four
    # letter blocks along the cylinder surface, each seated into the shell.
    # Placed so every corner stays outside the cone the visor sweeps through.
    logo_az = math.radians(47.0)
    logo_z = -0.060  # jaw area, below the face opening bottom
    logo_y_axis, logo_z_axis = _cylinder_decal_axes(logo_az)
    logo_base = _cylinder_decal_origin(logo_az, logo_z, 0.1495)
    for i in range(4):
        dy = -0.0180 + 0.0120 * i
        shell.visual(
            Box((0.0015, 0.0090, 0.014)),
            origin=Origin(
                xyz=(
                    logo_base.xyz[0] + dy * logo_y_axis[0],
                    logo_base.xyz[1] + dy * logo_y_axis[1],
                    logo_base.xyz[2] + dy * logo_z_axis[2],
                ),
                rpy=logo_base.rpy,
            ),
            material=red,
            name=f"nasa_logo_letter_{i}",
        )

    # US flag decal beside the pivot disk: white base, blue canton, red stripes.
    flag_az = math.radians(65.0)
    flag_z = -0.005  # near pivot height on the cylinder side
    shell.visual(
        Box((0.0015, 0.030, 0.020)),
        origin=_cylinder_decal_origin(flag_az, flag_z, 0.1495),
        material=frame_white,
        name="flag_decal_base",
    )
    y_axis, z_axis = _cylinder_decal_axes(flag_az)
    base = _cylinder_decal_origin(flag_az, flag_z, 0.1503)

    def _flag_offset(dy: float, dz: float) -> tuple[float, float, float]:
        return (
            base.xyz[0] + dy * y_axis[0] + dz * z_axis[0],
            base.xyz[1] + dy * y_axis[1] + dz * z_axis[1],
            base.xyz[2] + dy * y_axis[2] + dz * z_axis[2],
        )

    shell.visual(
        Box((0.0012, 0.012, 0.009)),
        origin=Origin(xyz=_flag_offset(-0.0085, 0.0053), rpy=base.rpy),
        material=flag_blue,
        name="flag_decal_canton",
    )
    for i in range(3):
        shell.visual(
            Box((0.0012, 0.020, 0.0028)),
            origin=Origin(xyz=_flag_offset(0.0045, 0.0068 - 0.0068 * i), rpy=base.rpy),
            material=flag_stripes,
            name=f"flag_decal_stripe_{i}",
        )

    # --- Visor assembly (flip-up child) ------------------------------------
    # Authored around the sphere center so the pivot axis passes through it:
    # rotating about Y keeps the bubble concentric with the shell.
    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_build_visor_frame_shape(), "visor_frame"),
        material=frame_white,
        name="visor_frame",
    )
    visor.visual(
        mesh_from_cadquery(_build_visor_bubble_shape(), "visor_bubble"),
        material=amber,
        name="visor_bubble",
    )
    for i, side in enumerate((1.0, -1.0)):
        visor.visual(
            mesh_from_cadquery(_build_pivot_knob_shape(side), f"pivot_knob_{i}"),
            material=gray,
            name=f"pivot_knob_{i}",
        )

    model.articulation(
        "visor_pivot",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=visor,
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        # Bubble extends along local +X; -Y makes positive q flip it upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.31),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    visor = object_model.get_part("visor")
    pivot = object_model.get_articulation("visor_pivot")

    # Hero features exist.
    bubble = visor.get_visual("visor_bubble")
    frame = visor.get_visual("visor_frame")
    lip = shell.get_visual("interior_lip")
    logo = shell.get_visual("nasa_logo_letter_0")
    ctx.check(
        "hero visuals present",
        all(v is not None for v in (bubble, frame, lip, logo)),
        details="missing one of visor_bubble/visor_frame/interior_lip/nasa_logo_letter_0",
    )

    # Amber bubble is transparent (alpha < 1) per the reference photo.
    amber = next(m for m in object_model.materials if m.name == "visor_amber")
    ctx.check(
        "visor bubble is transparent amber",
        amber.rgba is not None and amber.rgba[3] < 0.9,
        details=f"visor_amber rgba={amber.rgba}",
    )

    # Closed pose: the bubble covers the face opening (projected footprint) and
    # protrudes past the shell front.
    with ctx.pose({pivot: 0.0}):
        ctx.expect_overlap(
            visor,
            shell,
            axes="yz",
            elem_a="visor_bubble",
            elem_b="shell_body",
            min_overlap=0.14,
            name="closed bubble covers the face opening footprint",
        )
        shell_aabb = ctx.part_world_aabb(shell)
        shell_body_aabb = ctx.part_element_world_aabb(shell, elem="shell_body")
        closed_bubble = ctx.part_element_world_aabb(visor, elem="visor_bubble")
        ctx.check(
            "closed bubble protrudes past the shell face",
            shell_body_aabb is not None
            and closed_bubble is not None
            and closed_bubble[1][0] > shell_body_aabb[1][0] + 0.003,
            details=f"bubble={closed_bubble}, shell_body={shell_body_aabb}",
        )

    # Pivot disks are seated on the shell sides (the visor's mount path).
    ctx.allow_overlap(
        shell,
        visor,
        elem_a="shell_body",
        elem_b="visor_frame",
        reason="Pivot disks intentionally seat onto the cylinder barrel so the "
        "flip-visor hinge reads as mounted on the shell surface.",
    )
    ctx.expect_contact(
        visor,
        shell,
        elem_a="visor_frame",
        elem_b="shell_body",
        name="pivot disks seat on the shell",
    )

    # Pivot disks straddle the shell along the pivot axis (retained hinge look).
    ctx.expect_overlap(
        visor,
        shell,
        axes="y",
        elem_a="visor_frame",
        elem_b="shell_body",
        min_overlap=0.25,
        name="visor frame spans the shell across the pivot axis",
    )

    # Open pose: positive q flips the bubble up over the crown and back.
    with ctx.pose({pivot: 1.2}):
        open_bubble = ctx.part_element_world_aabb(visor, elem="visor_bubble")
    ctx.check(
        "visor bubble flips upward when opened",
        closed_bubble is not None
        and open_bubble is not None
        and open_bubble[1][2] > closed_bubble[1][2] + 0.03
        and open_bubble[0][2] > closed_bubble[0][2] + 0.10,
        details=f"closed={closed_bubble}, open={open_bubble}",
    )

    # Neck ring rests on the ground plane.
    ctx.check(
        "neck ring sits on the ground",
        shell_aabb is not None and abs(shell_aabb[0][2]) < 0.003,
        details=f"shell aabb={shell_aabb}",
    )

    # Decals stay tight to the cylinder surface (tangent, not floating away).
    logo_aabb = ctx.part_element_world_aabb(shell, elem="nasa_logo_letter_0")
    if logo_aabb is not None:
        logo_cx = 0.5 * (logo_aabb[0][0] + logo_aabb[1][0])
        logo_cy = 0.5 * (logo_aabb[0][1] + logo_aabb[1][1])
        logo_radial = math.sqrt(logo_cx ** 2 + logo_cy ** 2)
    else:
        logo_radial = None
    ctx.check(
        "nasa logo decal hugs the cylinder jaw surface",
        logo_radial is not None and abs(logo_radial - R_OUT) < 0.006,
        details=f"logo radial dist={logo_radial}, expected ~{R_OUT}",
    )

    # Shell body is a cylindrical barrel with domed top, not a sphere:
    # the Z extent must be significantly taller than the XY diameter.
    shell_body_aabb_check = ctx.part_element_world_aabb(shell, elem="shell_body")
    if shell_body_aabb_check is not None:
        sb_min, sb_max = shell_body_aabb_check
        sb_dz = sb_max[2] - sb_min[2]
        sb_dx = sb_max[0] - sb_min[0]
    else:
        sb_dz = sb_dx = 0.0
    ctx.check(
        "shell_body is a cylindrical barrel (taller than wide)",
        sb_dz > 0.350 and sb_dz > sb_dx + 0.040,
        details=f"shell_body dz={sb_dz:.4f}, dx={sb_dx:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
