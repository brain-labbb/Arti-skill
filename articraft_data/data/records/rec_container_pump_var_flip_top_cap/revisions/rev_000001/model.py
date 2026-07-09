from __future__ import annotations

# Clear soap-dispenser bottle with a flip-top disc cap closure.
# Forked from the press-down pump variant: the bottle body, shoulder, neck,
# and threaded collar are preserved. Instead of a pump head / swivel / stem /
# spout / dip tube, the collar now carries a hinged flip-top disc cap that
# swings open and shut on a single REVOLUTE hinge at the cap rim, revealing
# a dispensing orifice on the collar top.
#
# Frame: bottle main axis along +Z (vertical). Bottle bottom sits at z=0.
# Joint chain: bottle(fixed) -> collar (FIXED) -> flip_cap (REVOLUTE hinge)

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- key heights along +Z ----
BOTTLE_BOTTOM = 0.0
BOTTLE_BODY_TOP = 0.130  # where the cylindrical body ends and the shoulder starts
SHOULDER_TOP = 0.150  # top of the shoulder, base of the neck
NECK_TOP = 0.168  # top of the bottle neck (collar threads onto this)
BODY_R = 0.030  # bottle body outer radius (~0.06 m diameter)
NECK_R = 0.0150  # bottle neck outer radius

COLLAR_R = 0.0185  # white collar outer radius (wraps over the neck)
COLLAR_BOTTOM = 0.150
COLLAR_TOP = 0.176

# ---- flip cap geometry ----
CAP_RADIUS = 0.0165  # cap disc radius (slightly smaller than collar inner bore)
CAP_THICKNESS = 0.003  # cap disc thickness
LUG_WIDTH = 0.004  # hinge lug width along X
LUG_HEIGHT = 0.006  # hinge lug height above collar top
LUG_DEPTH = 0.005  # hinge lug depth along Y
HINGE_Y = COLLAR_R - 0.002  # hinge pin Y position (just inside the collar rim)
HINGE_Z = COLLAR_TOP + LUG_HEIGHT / 2.0  # hinge pin center (mid-height of the lugs)
HINGE_PIN_R = 0.0012  # hinge pin radius
HINGE_PIN_LEN = 0.022  # hinge pin length (along X, passes through both lugs)
ORIFICE_R = 0.004  # dispensing orifice radius
ORIFICE_HEIGHT = 0.004  # dispensing orifice height above collar top


def _bottle_mesh():
    """Transparent hollow bottle: lathe shell with domed base and narrow neck."""
    wall = 0.0022
    outer = [
        (0.0, BOTTLE_BOTTOM),
        (BODY_R, BOTTLE_BOTTOM + 0.004),
        (BODY_R, BOTTLE_BODY_TOP),
        (NECK_R + 0.004, SHOULDER_TOP),
        (NECK_R, SHOULDER_TOP + 0.004),
        (NECK_R, NECK_TOP),
    ]
    inner = [
        (0.0, BOTTLE_BOTTOM + 0.006),
        (BODY_R - wall, BOTTLE_BOTTOM + 0.010),
        (BODY_R - wall, BOTTLE_BODY_TOP),
        (NECK_R + 0.004 - wall, SHOULDER_TOP),
        (NECK_R - wall, SHOULDER_TOP + 0.004),
        (NECK_R - wall, NECK_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "bottle_shell")


def _neck_threads_mesh():
    """Thin thread ridges on the bottle neck for the screw collar."""
    geo = None
    for i, zz in enumerate((0.154, 0.160, 0.166)):
        ring = TorusGeometry(NECK_R + 0.0005, 0.0012, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zz)
        geo = ring if geo is None else geo.merge(ring)
    return mesh_from_geometry(geo, "neck_threads")


def _collar_mesh():
    """White threaded collar shell with hinge lugs, top plate, and dispensing orifice."""
    # Main collar shell (same as parent)
    outer = [
        (0.0, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_TOP),
        (NECK_R + 0.0015, COLLAR_TOP),
    ]
    inner = [
        (0.0, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)

    # Top plate: a thin annular disc at the collar top that closes the bore
    # and connects the outer wall to the orifice base. This is the surface
    # the flip cap seats on when closed.
    plate_thickness = 0.0015
    plate_inner = ORIFICE_R - 0.0005  # overlap with orifice outer edge for connectivity
    plate = LatheGeometry(
        [
            (plate_inner, COLLAR_TOP),
            (COLLAR_R - 0.001, COLLAR_TOP),
            (COLLAR_R - 0.001, COLLAR_TOP + plate_thickness),
            (plate_inner, COLLAR_TOP + plate_thickness),
        ],
        segments=36,
    )
    geo = geo.merge(plate)

    # Hinge lugs: two small bosses rising from the collar top rim at +Y.
    # Each lug extends down into the top plate for geometric connectivity.
    lug_x_offset = 0.006  # fixed offset so lugs straddle the cap tab at center
    for sign in (-1.0, 1.0):
        lug = BoxGeometry((LUG_WIDTH, LUG_DEPTH, LUG_HEIGHT + 0.002))
        lug.translate(
            sign * lug_x_offset,
            HINGE_Y,
            COLLAR_TOP + (LUG_HEIGHT + 0.002) / 2.0 - 0.001,
        )
        geo = geo.merge(lug)

    # Hinge pin: runs along X through the lugs. Merged into the collar shell
    # for connectivity (the pin is fixed to the collar).
    pin = CylinderGeometry(HINGE_PIN_R, HINGE_PIN_LEN, radial_segments=12)
    pin.rotate_y(math.pi / 2.0)  # align along X first
    pin.translate(0.0, HINGE_Y, HINGE_Z)  # then move to position
    geo = geo.merge(pin)

    # Dispensing orifice: a small raised nozzle centered on the collar top plate.
    # Extends slightly below the plate for connectivity.
    orifice = CylinderGeometry(ORIFICE_R, ORIFICE_HEIGHT + 0.002, radial_segments=16)
    orifice.translate(0.0, 0.0, COLLAR_TOP + (ORIFICE_HEIGHT + 0.002) / 2.0 - 0.001)
    geo = geo.merge(orifice)

    return mesh_from_geometry(geo, "collar_shell")


def _collar_knurl_mesh():
    """Knurl ribs around the collar for grip texture."""
    geo = None
    h = COLLAR_TOP - COLLAR_BOTTOM
    for i in range(22):
        ang = 2 * math.pi * i / 22
        rib = CylinderGeometry(0.0012, h, radial_segments=6)
        rib.translate(COLLAR_R - 0.0002, 0.0, COLLAR_BOTTOM + h / 2.0)
        rib.rotate_z(ang)
        geo = rib if geo is None else geo.merge(rib)
    return mesh_from_geometry(geo, "collar_knurl")


def _cap_disc_mesh():
    """Flip-top disc cap: a flat disc with a hinge tab and a thumb tab.

    The cap local frame origin is at the hinge point (matching the articulation
    frame at HINGE_Z height). The disc body extends toward -Y from the hinge
    and sits below the origin (on the collar top surface).
    """
    # Disc center in local frame:
    # Y: disc center is at -HINGE_Y (centered on the bottle axis)
    # Z: disc bottom sits on collar top, so center Z = COLLAR_TOP + CAP_THICKNESS/2 - HINGE_Z
    disc_center_y = -HINGE_Y
    disc_center_z = COLLAR_TOP + CAP_THICKNESS / 2.0 - HINGE_Z

    # Use a lathe for the disc to get a nice rounded edge
    disc_profile = [
        (0.0, disc_center_z - CAP_THICKNESS / 2.0),
        (CAP_RADIUS - 0.001, disc_center_z - CAP_THICKNESS / 2.0),
        (CAP_RADIUS, disc_center_z - CAP_THICKNESS / 2.0 + 0.0005),
        (CAP_RADIUS, disc_center_z + CAP_THICKNESS / 2.0 - 0.0005),
        (CAP_RADIUS - 0.001, disc_center_z + CAP_THICKNESS / 2.0),
        (0.0, disc_center_z + CAP_THICKNESS / 2.0),
    ]
    geo = LatheGeometry(disc_profile, segments=36)
    geo.translate(0.0, disc_center_y, 0.0)

    # Hinge tab: a small tongue at the +Y edge that fits between the collar lugs.
    # In local frame, the hinge tab is near the origin (at the hinge point).
    # It should fill the gap between the lugs vertically.
    tab = BoxGeometry((0.008, 0.005, LUG_HEIGHT - 0.001))
    tab.translate(0.0, 0.002, 0.0)
    geo = geo.merge(tab)

    # Thumb tab: a small lip at the -Y edge for flipping the cap open.
    thumb = BoxGeometry((0.008, 0.005, 0.005))
    thumb.translate(0.0, disc_center_y - CAP_RADIUS - 0.001, disc_center_z + 0.001)
    geo = geo.merge(thumb)

    return mesh_from_geometry(geo, "cap_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser_flip_cap")

    clear = model.material("clear_plastic", rgba=(0.74, 0.80, 0.82, 0.25))
    label_mat = model.material("label", rgba=(0.96, 0.96, 0.94, 1.0))
    white = model.material("cap_white", rgba=(0.93, 0.93, 0.94, 1.0))

    # ---- bottle (root): clear shell + label band + neck threads ----
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")

    # Wrap-around label band on the lower body.
    label = LatheGeometry.from_shell_profiles(
        [
            (BODY_R + 0.0008, 0.040),
            (BODY_R + 0.0008, 0.110),
        ],
        [
            (BODY_R - 0.0002, 0.040),
            (BODY_R - 0.0002, 0.110),
        ],
        segments=48,
    )
    bottle.visual(mesh_from_geometry(label, "label_band"), material=label_mat, name="label_band")
    bottle.visual(_neck_threads_mesh(), material=clear, name="neck_threads")

    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BOTTLE_BODY_TOP),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_BODY_TOP / 2.0)),
    )

    # ---- collar: white threaded cap with hinge lugs and orifice ----
    collar = model.part("collar")
    collar.visual(_collar_mesh(), material=white, name="collar_shell")
    collar.visual(_collar_knurl_mesh(), material=white, name="collar_knurl")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - COLLAR_BOTTOM),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_BOTTOM + COLLAR_TOP) / 2.0)),
    )
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- flip_cap: disc cap that hinges open/closed at the collar rim ----
    flip_cap = model.part("flip_cap")
    flip_cap.visual(_cap_disc_mesh(), material=white, name="cap_disc")
    flip_cap.inertial = Inertial.from_geometry(
        Cylinder(CAP_RADIUS, CAP_THICKNESS),
        mass=0.008,
        origin=Origin(xyz=(0.0, -HINGE_Y, CAP_THICKNESS / 2.0)),
    )

    # REVOLUTE hinge: cap swings open about the hinge pin at the +Y collar rim.
    # Origin at the hinge pin center in the collar (parent) frame.
    # Axis = (-1, 0, 0) so positive rotation lifts the -Y edge of the cap upward.
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=flip_cap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=2.4
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    flip_cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("cap_hinge")

    # --- bottle is transparent ---
    shell_vis = bottle.get_visual("bottle_shell")
    rgba = shell_vis.material.rgba
    ctx.check(
        "bottle is transparent",
        rgba is not None and rgba[3] < 1.0,
        details=f"bottle_shell rgba={rgba}",
    )

    # --- collar is seated on the bottle neck (intentional thread overlap) ---
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="neck_threads",
        reason="The white collar is threaded over the bottle neck; the threads engage inside the collar.",
    )
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        reason="The collar skirt wraps over the bottle neck wall.",
    )
    ctx.expect_overlap(
        collar, bottle, axes="z", min_overlap=0.005, name="collar seated over the neck"
    )

    # --- flip cap exists and has the disc visual ---
    cap_vis = flip_cap.get_visual("cap_disc")
    ctx.check(
        "flip cap has cap_disc visual",
        cap_vis is not None,
        details="cap_disc visual missing from flip_cap part",
    )

    # --- dispensing orifice is present on the collar ---
    collar_shell_vis = collar.get_visual("collar_shell")
    ctx.check(
        "collar has dispensing orifice geometry",
        collar_shell_vis is not None,
        details="collar_shell visual missing",
    )

    # --- hinge joint is REVOLUTE ---
    ctx.check(
        "cap_hinge is REVOLUTE",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"cap_hinge type={hinge.articulation_type}",
    )

    # --- closed pose: cap sits on the collar top ---
    with ctx.pose({hinge: 0.0}):
        cap_pos = ctx.part_world_position(flip_cap)
        ctx.check(
            "closed cap is near collar top height",
            cap_pos is not None and abs(cap_pos[2] - HINGE_Z) < 0.010,
            details=f"cap z={cap_pos[2] if cap_pos else None}, hinge z={HINGE_Z}",
        )

    # --- open pose: cap swings upward when hinge opens ---
    rest_aabb = ctx.part_world_aabb(flip_cap)
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(flip_cap)
    rest_max_z = rest_aabb[1][2] if rest_aabb else None
    open_max_z = open_aabb[1][2] if open_aabb else None
    ctx.check(
        "flip cap opens upward (hinge positive pose raises cap)",
        rest_max_z is not None and open_max_z is not None and open_max_z > rest_max_z + 0.010,
        details=f"rest_max_z={rest_max_z}, open_max_z={open_max_z}",
    )

    # --- cap hinge tab overlaps with collar hinge region (intentional, seated) ---
    ctx.allow_overlap(
        flip_cap,
        collar,
        elem_a="cap_disc",
        elem_b="collar_shell",
        reason="The cap hinge tab nests between the collar hinge lugs at the rim, and the cap rotates around the hinge pin integrated into the collar.",
    )

    # --- cap contact with collar when closed (seated) ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            flip_cap,
            collar,
            elem_a="cap_disc",
            elem_b="collar_shell",
            name="closed cap seats on collar top",
        )

    return ctx.report()


object_model = build_object_model()
