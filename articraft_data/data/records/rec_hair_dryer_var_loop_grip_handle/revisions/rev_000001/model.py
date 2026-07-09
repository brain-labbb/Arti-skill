from __future__ import annotations

# Pink compact hair dryer — open-loop grip handle variant.
# Frame: barrel axis along +X (front/nozzle at +X, rear intake at -X),
# barrel centerline at z=0, handle hanging down (-Z).
# Handle: open-loop grip frame with oval hand cutout and integrated switch shelf.
# Articulations:
#   - concentrator nozzle: CONTINUOUS spin about the barrel axis (orient the slot)
#   - two slide switches on the shelf: PRISMATIC fore/aft travel
#   - power cord + plug: FIXED (flexible cable, modeled as one drooping tube)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

BARREL_FRONT_X = 0.175
NOZZLE_MOUNT_X = 0.163
HANDLE_CENTER_X = 0.063

# Shelf and switch mounting constants.
# The handle +Y skin is at y ≈ 0.019-0.023 depending on Z.  The barrel inner
# surface at x ≈ 0.060 sits at y ≈ 0.034, so switch hardware must stay below
# y ≈ 0.030 to clear the barrel shell mesh.
FACE_PLATE_Y_DEPTH = 0.004
FACE_PLATE_Y_CENTER = 0.021         # face plate straddles the handle +Y skin
FACE_PLATE_Y_OUTER = FACE_PLATE_Y_CENTER + FACE_PLATE_Y_DEPTH / 2.0  # 0.023
SWITCH_NUB_HALF_Y = 0.0025          # thin slider nub (0.005 total depth)
SWITCH_MOUNT_Y = FACE_PLATE_Y_OUTER  # joint origin at the face-plate contact surface

SWITCH_Z_POSITIONS = (-0.018, -0.046)


def _loft(sections) -> cq.Workplane:
    # sections: list of ("circle", x, r) or ("rect", x, w, h) along +X (YZ planes).
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            wp = wp.circle(s[2])
        else:
            wp = wp.rect(s[2], s[3])
        prev = x
    return wp.loft(ruled=False)


def _barrel_solid() -> cq.Workplane:
    # Hollow housing: outer loft minus a slightly smaller inner loft that pokes
    # past both ends, so the barrel reads as a real open-ended shell.
    outer = _loft(
        [
            ("circle", 0.0, 0.037),
            ("circle", 0.030, 0.042),
            ("circle", 0.100, 0.040),
            ("circle", 0.150, 0.033),
            ("circle", 0.175, 0.030),
        ]
    )
    inner = _loft(
        [
            ("circle", -0.006, 0.034),
            ("circle", 0.030, 0.039),
            ("circle", 0.100, 0.037),
            ("circle", 0.150, 0.030),
            ("circle", 0.181, 0.027),
        ]
    )
    return outer.cut(inner)


def _handle_solid() -> cq.Workplane:
    """Open-loop grip handle: solid frame with oval hand cutout."""

    # Outer frame: lofted from barrel junction down to base.
    # Wider in Y than the parent handle so the frame walls around the oval cutout
    # remain structurally credible (~10 mm each side).
    outer = (
        cq.Workplane("XY")
        .center(HANDLE_CENTER_X, 0.0)
        .rect(0.056, 0.038)        # z = 0.000  blend with barrel underside
        .workplane(offset=-0.025)
        .rect(0.052, 0.044)        # z = -0.025  upper grip widens for frame
        .workplane(offset=-0.045)
        .rect(0.050, 0.046)        # z = -0.070  mid grip (widest around cutout)
        .workplane(offset=-0.045)
        .rect(0.044, 0.036)        # z = -0.115  base tapers back in
        .loft(ruled=False)
    )

    # Oval hand cutout: through-hole in Y for the finger grip.
    # CadQuery XZ workplane → local-x = world X, local-y = world Z,
    # extrude direction = world Y.
    cutout = (
        cq.Workplane("XZ")
        .center(HANDLE_CENTER_X, -0.072)
        .ellipse(0.014, 0.022)     # ~28 mm wide × ~44 mm tall
        .extrude(0.060, both=True) # well past the handle Y extent
    )
    return outer.cut(cutout)


def _switch_nub():
    """Shared geometry helper: identical slider nub for every switch."""
    return Box((0.013, SWITCH_NUB_HALF_Y * 2.0, 0.011))


def _nozzle_mesh():
    # Concentrator: round back (slips over the barrel lip) lofted to a wide thin
    # slot, hollowed so air passes through.
    outer = _loft(
        [
            ("circle", 0.0, 0.033),
            ("rect", 0.028, 0.070, 0.030),
            ("rect", 0.052, 0.082, 0.013),
        ]
    )
    inner = _loft(
        [
            ("circle", -0.006, 0.030),
            ("rect", 0.028, 0.064, 0.024),
            ("rect", 0.058, 0.078, 0.008),
        ]
    )
    noz = outer.cut(inner)
    return mesh_from_cadquery(noz, "nozzle")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hair_dryer")

    shell_pink = model.material("shell_pink", rgba=(0.96, 0.71, 0.78, 1.0))
    dark = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    switch_gray = model.material("switch_gray", rgba=(0.32, 0.32, 0.34, 1.0))

    # ---- body (root): barrel + handle + rear filter cap + switch shelf ----
    body = model.part("body")

    body_shell = _barrel_solid().union(_handle_solid())
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell"),
        material=shell_pink,
        name="body_shell",
    )

    # Rear intake filter cap with concentric grille ribs.
    cap = CylinderGeometry(0.037, 0.012, radial_segments=48).rotate_y(math.pi / 2.0)
    cap.translate(-0.004, 0.0, 0.0)
    for rr in (0.014, 0.022, 0.030):
        ring = TorusGeometry(
            rr, 0.0016, radial_segments=10, tubular_segments=40
        ).rotate_y(math.pi / 2.0)
        ring.translate(-0.011, 0.0, 0.0)
        cap.merge(ring)
    body.visual(
        mesh_from_geometry(cap, "rear_filter"), material=dark, name="rear_filter"
    )

    # Integrated switch shelf: dark face plate on the +Y face of the handle.
    body.visual(
        Box((0.032, FACE_PLATE_Y_DEPTH, 0.056)),
        origin=Origin(
            xyz=(HANDLE_CENTER_X - 0.003, FACE_PLATE_Y_CENTER, -0.032)
        ),
        material=dark,
        name="switch_shelf",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.20, 0.085, 0.085)), mass=0.45, origin=Origin(xyz=(0.085, 0.0, 0.0))
    )

    # ---- concentrator nozzle: spins about the barrel axis ----
    nozzle = model.part("nozzle")
    nozzle.visual(_nozzle_mesh(), material=dark, name="nozzle_shell")
    nozzle.inertial = Inertial.from_geometry(
        Box((0.05, 0.082, 0.07)), mass=0.04, origin=Origin(xyz=(0.025, 0.0, 0.0))
    )
    model.articulation(
        "barrel_to_nozzle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(NOZZLE_MOUNT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    # ---- two slide switches on the shelf, prismatic fore/aft ----
    for i in range(len(SWITCH_Z_POSITIONS)):
        name = f"switch_{i}"
        sz = SWITCH_Z_POSITIONS[i]
        sw = model.part(name)
        sw.visual(
            _switch_nub(),
            # Nub center is SWITCH_NUB_HALF_Y above the part/joint origin so the
            # nub inner face sits flush on the face-plate contact surface.
            origin=Origin(xyz=(0.0, SWITCH_NUB_HALF_Y, 0.0)),
            material=switch_gray,
            name=f"{name}_nub",
        )
        sw.inertial = Inertial.from_geometry(_switch_nub(), mass=0.003)
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=sw,
            origin=Origin(xyz=(HANDLE_CENTER_X - 0.003, SWITCH_MOUNT_Y, sz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.1, lower=-0.007, upper=0.007
            ),
        )

    # ---- power cord + plug: one drooping flexible cable, fixed to the handle ----
    cord_pts = [
        (0.045, 0.0, -0.116),
        (0.052, 0.0, -0.150),
        (0.030, 0.0, -0.190),
        (-0.020, 0.0, -0.208),
        (-0.060, 0.0, -0.208),
        (-0.090, 0.0, -0.208),
    ]
    cord = tube_from_spline_points(
        cord_pts, radius=0.0042, samples_per_segment=16, radial_segments=14
    )
    # Strain-relief sleeve where the cord exits the handle base.
    relief = CylinderGeometry(0.0085, 0.024).translate(0.045, 0.0, -0.118)
    cord.merge(relief)
    # Plug body + two pins at the cord end.
    plug = BoxGeometry((0.040, 0.028, 0.022)).translate(-0.090, 0.0, -0.208)
    cord.merge(plug)
    for i in range(2):
        py = (-0.008, 0.008)[i]
        pin = CylinderGeometry(0.0035, 0.020).rotate_y(math.pi / 2.0)
        pin.translate(-0.119, py, -0.208)
        cord.merge(pin)

    power_cord = model.part("power_cord")
    power_cord.visual(
        mesh_from_geometry(cord, "power_cord"), material=dark, name="cord_shell"
    )
    power_cord.inertial = Inertial.from_geometry(Box((0.18, 0.03, 0.12)), mass=0.06)
    model.articulation(
        "body_to_cord",
        ArticulationType.FIXED,
        parent=body,
        child=power_cord,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    nozzle = object_model.get_part("nozzle")
    switch_0 = object_model.get_part("switch_0")
    switch_1 = object_model.get_part("switch_1")
    cord = object_model.get_part("power_cord")
    spin = object_model.get_articulation("barrel_to_nozzle")
    switch_0_joint = object_model.get_articulation("body_to_switch_0")

    # ---- Handle: open-loop grip with oval cutout and integrated switch shelf ----
    # The grip frame extends well below the barrel for the finger opening.
    body_aabb = ctx.part_world_aabb(body)
    body_z_extent = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "handle grip extends below barrel for finger frame",
        body_z_extent > 0.12,
        details=f"body Z extent={body_z_extent:.4f}",
    )

    # Both switches rest on the shelf face plate (contact at the mounting surface).
    ctx.expect_contact(
        switch_0, body, name="switch_0 rests on handle shelf"
    )
    ctx.expect_contact(
        switch_1, body, name="switch_1 rests on handle shelf"
    )

    # Switches are vertically separated (power above heat) on the shelf.
    sw0_z = ctx.part_world_position(switch_0)[2]
    sw1_z = ctx.part_world_position(switch_1)[2]
    ctx.check(
        "switches are vertically separated on shelf",
        sw0_z > sw1_z + 0.010,
        details=f"switch_0 z={sw0_z:.4f}, switch_1 z={sw1_z:.4f}",
    )

    # ---- Nozzle: seated over barrel front and spins correctly ----
    ctx.allow_overlap(
        nozzle,
        body,
        elem_a="nozzle_shell",
        elem_b="body_shell",
        reason="Concentrator back rim is intentionally slipped over the barrel front lip.",
    )
    ctx.expect_overlap(
        nozzle, body, axes="x", min_overlap=0.006, name="nozzle seated over barrel front"
    )
    noz_pos = ctx.part_world_position(nozzle)
    ctx.check(
        "nozzle mounted at the barrel front",
        noz_pos is not None and noz_pos[0] > 0.15,
        details=f"nozzle origin={noz_pos}",
    )

    # Spinning the nozzle reorients the flat slot: wide axis flips from Y to Z.
    ext0 = _ext(ctx.part_world_aabb(nozzle))
    ctx.check(
        "slot is horizontal at rest",
        ext0[1] > ext0[2] + 0.005,
        details=f"rest extents={ext0}",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ext90 = _ext(ctx.part_world_aabb(nozzle))
    ctx.check(
        "nozzle spin rotates the slot toward vertical",
        ext90[2] > ext90[1] + 0.005,
        details=f"quarter-turn extents={ext90}",
    )

    # ---- Switches slide fore/aft on the shelf ----
    rest_x = ctx.part_world_position(switch_0)[0]
    with ctx.pose({switch_0_joint: 0.007}):
        slid_x = ctx.part_world_position(switch_0)[0]
    ctx.check(
        "switch_0 slides forward on shelf",
        slid_x > rest_x + 0.004,
        details=f"rest_x={rest_x}, slid_x={slid_x}",
    )

    # ---- Cord: attached at handle base ----
    ctx.allow_overlap(
        body,
        cord,
        elem_a="body_shell",
        elem_b="cord_shell",
        reason="Strain-relief sleeve and cord top intentionally enter the handle base.",
    )
    ctx.expect_contact(cord, body, name="cord attached to handle")

    return ctx.report()


object_model = build_object_model()
