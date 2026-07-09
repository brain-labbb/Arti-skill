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

# Costume-style NASA astronaut helmet with sliding sunshade (shuttle-era).
# White hollow spherical shell with a circular front opening, stepped ribbed
# neck-ring base, rear ribbed hose fitting, black interior chin lip, red NASA
# worm-logo decal and US flag decal, plus a sliding amber bubble visor carried
# by a white frame whose slider blocks ride in vertical guide rails on the
# shell sides.  The visor drops straight down the shell front to close and
# slides up to retract (PRISMATIC joint along +Z).

# NOTE: booleans are done on cq Shapes (.val()) rather than Workplanes; the
# Workplane-level intersect() produced corrupt results for these sphere/cyl
# combinations in this CadQuery build.

# Global layout (meters).
R_OUT = 0.150       # shell outer radius
R_IN = 0.142        # shell inner radius (hollow)
CENTER_Z = 0.160    # sphere center height above ground
OPENING_R = 0.092   # circular face-opening radius (cut along +X)
OPENING_ZC = 0.010  # opening/visor axis height above the sphere center
NECK_CUT_Z = -0.105 # sphere is cut flat below this local z
VISOR_R_IN = 0.152  # amber bubble inner radius (2 mm clear of shell)
VISOR_R_OUT = 0.157
FRAME_R_IN = 0.154  # raised white frame sits proud of the bubble
FRAME_R_OUT = 0.161

# Guide-rail parameters (shell-local coordinates, sphere center at origin).
# Two vertical rails on the shell exterior, one per side of the face opening.
RAIL_Y = 0.105       # Y center of each rail (well outside bubble cyl radius 0.094)
RAIL_Z_LO = -0.020   # bottom of rail
RAIL_Z_HI = 0.110    # top of rail
RAIL_Z_LEN = RAIL_Z_HI - RAIL_Z_LO
RAIL_Z_CENTER = 0.5 * (RAIL_Z_LO + RAIL_Z_HI)
RAIL_WIDTH_Y = 0.014 # rail width in Y

# Slider-block parameters (visor-local = shell-local at q = 0).
SLIDER_X = 0.120     # X center of slider block (outside the rail)
SLIDER_THICK_X = 0.008
SLIDER_WIDTH_Y = 0.013
SLIDER_HEIGHT_Z = 0.028

# Prismatic travel (meters).  At q = 0 the visor is closed (covering the face
# opening); at q = SLIDE_UPPER the visor has retracted upward.
SLIDE_UPPER = 0.080


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


def _build_shell_shape() -> cq.Shape:
    """White helmet shell: hollow sphere, face opening, stepped ribbed collar,
    rear ribbed hose fitting, and two vertical guide rails for the sliding
    sunshade."""
    shell = _sphere_shell(R_OUT, R_IN)

    # Flat-cut the sphere bottom where the neck ring takes over.
    shell = shell.cut(_box(0.5, 0.5, 0.4, (0.0, 0.0, NECK_CUT_Z - 0.2)))

    # Circular face opening for the visor, cut along +X.
    shell = shell.cut(_x_cylinder(OPENING_R, 0.02, 0.25, OPENING_ZC))

    # Stepped neck-ring collar (upper tier + wider bottom tier), hollow inside.
    collar = _z_cylinder(0.112, -0.160, -0.100)
    collar = collar.fuse(_z_cylinder(0.118, -0.160, -0.140))
    collar = collar.cut(_z_cylinder(0.095, -0.165, -0.095))

    # Vertical rib notches around the upper collar rim.
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

    # Guide rails for the sliding sunshade: two curved tracks on the shell
    # exterior, one on each side of the face opening.  Each rail is a thin
    # sphere-shell strip that follows the shell curvature.
    for side in (1.0, -1.0):
        rail = _sphere_shell(R_OUT + 0.005, R_OUT - 0.001)
        rail = rail.intersect(
            _box(0.12, RAIL_WIDTH_Y, RAIL_Z_LEN,
                 (0.08, side * RAIL_Y, RAIL_Z_CENTER))
        )
        shell = shell.fuse(rail)

    return shell


def _build_interior_lip_shape() -> cq.Shape:
    """Black interior chin lip visible along the bottom of the face opening."""
    lip = _sphere_shell(0.147, 0.1405)
    lip = lip.intersect(_box(0.4, 0.4, 0.045, (0.0, 0.0, -0.0575)))
    return lip.intersect(_x_cylinder(0.096, 0.01, 0.20, OPENING_ZC))


def _build_visor_bubble_shape() -> cq.Shape:
    """Amber transparent bubble: spherical cap shell over the face opening."""
    return _sphere_shell(VISOR_R_OUT, VISOR_R_IN).intersect(
        _x_cylinder(0.094, 0.0, 0.20, OPENING_ZC)
    )


def _build_visor_frame_shape() -> cq.Shape:
    """White raised frame: bezel ring around the bubble, two curved side arms,
    and two slider blocks that ride in the shell guide rails (replacing the
    original round pivot disks with a prismatic slide interface)."""
    # Bezel ring following the sphere surface around the opening.
    annulus = _x_cylinder(0.104, 0.0, 0.20, OPENING_ZC).cut(
        _x_cylinder(0.076, -0.01, 0.21, OPENING_ZC)
    )
    frame = _sphere_shell(FRAME_R_OUT, FRAME_R_IN).intersect(annulus)

    for side in (1.0, -1.0):
        # Curved arm from the bezel outward to the slider-block position,
        # hugging the shell surface.  The arm sphere shell (0.151..0.163)
        # overlaps both the bezel ring (0.154..0.161) and the slider block.
        arm = _sphere_shell(0.163, 0.151).intersect(
            _box(0.07, 0.040, 0.025, (0.120, side * 0.093, OPENING_ZC))
        )
        frame = frame.fuse(arm)

        # Slider block at the rail Y position, centered on the face opening
        # height so the block engages the rail at q = 0 (closed pose).
        block = _box(
            SLIDER_THICK_X, SLIDER_WIDTH_Y, SLIDER_HEIGHT_Z,
            (SLIDER_X, side * RAIL_Y, OPENING_ZC),
        )
        frame = frame.fuse(block)

    return frame


def _tangent_decal_origin(az: float, elev: float, r: float) -> Origin:
    """Origin for a thin decal box tangent to the shell at azimuth/elevation."""
    x = r * math.cos(elev) * math.cos(az)
    y = r * math.cos(elev) * math.sin(az)
    z = r * math.sin(elev)
    return Origin(xyz=(x, y, z + CENTER_Z), rpy=(0.0, -elev, az))


def _decal_frame_axes(az: float, elev: float):
    """Local width (y) and height (z) axes of a tangent decal box."""
    y_axis = (-math.sin(az), math.cos(az), 0.0)
    z_axis = (
        -math.sin(elev) * math.cos(az),
        -math.sin(elev) * math.sin(az),
        math.cos(elev),
    )
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

    # Red NASA worm-logo decal on the jaw, below the visor opening.
    logo_az = math.radians(47.0)
    logo_el = math.radians(-32.0)
    logo_y_axis, logo_z_axis = _decal_frame_axes(logo_az, logo_el)
    logo_base = _tangent_decal_origin(logo_az, logo_el, 0.1495)
    for i in range(4):
        dy = -0.0180 + 0.0120 * i
        shell.visual(
            Box((0.0015, 0.0090, 0.014)),
            origin=Origin(
                xyz=(
                    logo_base.xyz[0] + dy * logo_y_axis[0],
                    logo_base.xyz[1] + dy * logo_y_axis[1],
                    logo_base.xyz[2] + dy * logo_y_axis[2],
                ),
                rpy=logo_base.rpy,
            ),
            material=red,
            name=f"nasa_logo_letter_{i}",
        )

    # US flag decal beside the guide rail: white base, blue canton, red stripes.
    flag_az = math.radians(65.0)
    flag_el = math.radians(-11.5)
    shell.visual(
        Box((0.0015, 0.030, 0.020)),
        origin=_tangent_decal_origin(flag_az, flag_el, 0.1495),
        material=frame_white,
        name="flag_decal_base",
    )
    y_axis, z_axis = _decal_frame_axes(flag_az, flag_el)
    base = _tangent_decal_origin(flag_az, flag_el, 0.1503)

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

    # --- Visor assembly (sliding sunshade child) ---------------------------
    # Authored at the sphere center so the prismatic slide axis (+Z) moves the
    # bubble and frame straight up/down relative to the face opening.
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

    # Small gray retaining cap on each slider block (visual detail).
    for i, side in enumerate((1.0, -1.0)):
        cap = _box(
            0.005, 0.010, 0.010,
            (SLIDER_X + 0.005, side * RAIL_Y, OPENING_ZC),
        )
        visor.visual(
            mesh_from_cadquery(cap, f"slider_cap_{i}"),
            material=gray,
            name=f"slider_cap_{i}",
        )

    model.articulation(
        "visor_slide",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=visor,
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        # +Z: positive q retracts the visor upward, exposing the face opening.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=0.15, lower=0.0, upper=SLIDE_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    visor = object_model.get_part("visor")
    slide = object_model.get_articulation("visor_slide")

    # --- Hero features exist -----------------------------------------------
    bubble = visor.get_visual("visor_bubble")
    frame = visor.get_visual("visor_frame")
    lip = shell.get_visual("interior_lip")
    logo = shell.get_visual("nasa_logo_letter_0")
    ctx.check(
        "hero visuals present",
        all(v is not None for v in (bubble, frame, lip, logo)),
        details="missing one of visor_bubble/visor_frame/interior_lip/nasa_logo_letter_0",
    )

    # --- The joint is PRISMATIC (core axis change) -------------------------
    ctx.check(
        "visor_slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {slide.articulation_type}",
    )

    # --- Amber bubble is transparent ----------------------------------------
    amber = next(m for m in object_model.materials if m.name == "visor_amber")
    ctx.check(
        "visor bubble is transparent amber",
        amber.rgba is not None and amber.rgba[3] < 0.9,
        details=f"visor_amber rgba={amber.rgba}",
    )

    # --- Closed pose (q = 0): bubble covers the face opening ---------------
    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            visor, shell,
            axes="yz",
            elem_a="visor_bubble", elem_b="shell_body",
            min_overlap=0.14,
            name="closed bubble covers the face opening footprint",
        )
        shell_body_aabb = ctx.part_element_world_aabb(shell, elem="shell_body")
        closed_bubble = ctx.part_element_world_aabb(visor, elem="visor_bubble")
        ctx.check(
            "closed bubble protrudes past the shell face",
            shell_body_aabb is not None
            and closed_bubble is not None
            and closed_bubble[1][0] > shell_body_aabb[1][0] + 0.02,
            details=f"bubble={closed_bubble}, shell_body={shell_body_aabb}",
        )

    # --- Slider blocks engage the guide rails ------------------------------
    # The slider blocks on the visor_frame intentionally overlap the shell
    # guide rails by ~4 mm to represent the sliding engagement.
    ctx.allow_overlap(
        shell, visor,
        elem_a="shell_body", elem_b="visor_frame",
        reason="Slider blocks intentionally seat ~4 mm into the guide rails "
        "so the prismatic slide reads as mechanically engaged; the embed is "
        "a local hidden interface on each side.",
    )
    ctx.expect_contact(
        visor, shell,
        elem_a="visor_frame", elem_b="shell_body",
        name="slider blocks seat in the guide rails",
    )

    # Visor frame spans the shell across the rail axis (Y).
    ctx.expect_overlap(
        visor, shell,
        axes="y",
        elem_a="visor_frame", elem_b="shell_body",
        min_overlap=0.16,
        name="visor frame spans the shell across the rail axis",
    )

    # --- Open pose (q = upper): visor slides upward ------------------------
    with ctx.pose({slide: SLIDE_UPPER}):
        open_bubble = ctx.part_element_world_aabb(visor, elem="visor_bubble")
    ctx.check(
        "visor bubble slides upward when retracted",
        closed_bubble is not None
        and open_bubble is not None
        and open_bubble[0][2] > closed_bubble[0][2] + 0.05
        and open_bubble[1][2] > closed_bubble[1][2] + 0.05,
        details=f"closed={closed_bubble}, open={open_bubble}",
    )

    # Prove that the slide axis actually moves the child in Z (not X or Y).
    with ctx.pose({slide: 0.0}):
        closed_center = ctx.part_element_world_aabb(visor, elem="visor_bubble")
    with ctx.pose({slide: SLIDE_UPPER}):
        open_center = ctx.part_element_world_aabb(visor, elem="visor_bubble")
    ctx.check(
        "visor_slide translates visor along Z only",
        closed_center is not None
        and open_center is not None
        and abs(
            0.5 * (open_center[0][0] + open_center[1][0])
            - 0.5 * (closed_center[0][0] + closed_center[1][0])
        )
        < 0.002
        and abs(
            0.5 * (open_center[0][1] + open_center[1][1])
            - 0.5 * (closed_center[0][1] + closed_center[1][1])
        )
        < 0.002,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    # --- Neck ring rests on the ground plane --------------------------------
    shell_aabb = ctx.part_world_aabb(shell)
    ctx.check(
        "neck ring sits on the ground",
        shell_aabb is not None and abs(shell_aabb[0][2]) < 0.003,
        details=f"shell aabb={shell_aabb}",
    )

    # --- Decals stay tight to the shell surface -----------------------------
    logo_aabb = ctx.part_element_world_aabb(shell, elem="nasa_logo_letter_0")
    ctx.check(
        "nasa logo decal hugs the jaw surface",
        logo_aabb is not None
        and math.dist(
            (
                0.5 * (logo_aabb[0][0] + logo_aabb[1][0]),
                0.5 * (logo_aabb[0][1] + logo_aabb[1][1]),
                0.5 * (logo_aabb[0][2] + logo_aabb[1][2]) - CENTER_Z,
            ),
            (0.0, 0.0, 0.0),
        )
        < R_OUT + 0.004,
        details=f"logo aabb={logo_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
