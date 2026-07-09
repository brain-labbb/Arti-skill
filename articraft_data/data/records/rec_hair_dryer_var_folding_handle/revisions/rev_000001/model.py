from __future__ import annotations

# Pink compact travel hair dryer with folding handle.
# Frame: barrel axis along +X (front/nozzle at +X, rear intake at -X),
# barrel centerline at z=0, handle folds from deployed (-Z) toward nozzle (+X).
# Articulations:
#   - body_to_handle: REVOLUTE hinge at barrel underside, axis along -Y
#     (positive q folds handle toward nozzle; q=0 is locked-open deployed)
#   - barrel_to_nozzle: CONTINUOUS spin about the barrel axis
#   - handle_to_power_switch: PRISMATIC fore/aft travel
#   - handle_to_heat_switch: PRISMATIC fore/aft travel
#   - handle_to_cord: FIXED (flexible cable, modeled as one drooping tube)

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
HINGE_X = 0.063
HINGE_Z = -0.041  # barrel bottom surface at the hinge location


def _loft(sections) -> cq.Workplane:
    """Loft along +X through YZ-plane sections."""
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
    """Hollow barrel housing: outer loft minus inner loft for open-ended shell."""
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
    """Folding handle in local frame: hinge at origin, grip extends -Z.
    Includes integrated hinge ear at top that wraps around the hinge barrel."""
    grip = (
        cq.Workplane("XY")
        .rect(0.050, 0.032)  # z=0 (top, wider for hinge area)
        .workplane(offset=-0.015)
        .rect(0.048, 0.030)  # z=-0.015 (transition below hinge)
        .workplane(offset=-0.030)
        .rect(0.046, 0.030)  # z=-0.045 (mid grip)
        .workplane(offset=-0.035)
        .rect(0.040, 0.032)  # z=-0.080 (bottom)
        .loft(ruled=False)
    )
    # Hinge ear: cylinder along Y at origin, protrudes beyond handle sides.
    ear = (
        cq.Workplane("XZ")
        .circle(0.009)
        .extrude(0.018, both=True)  # 36mm total along Y
    )
    return grip.union(ear)


def _nozzle_mesh():
    """Concentrator: round back lofted to wide thin slot, hollowed."""
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


def _rear_filter_geom():
    """Rear intake filter cap with concentric grille ribs."""
    cap = CylinderGeometry(0.037, 0.012, radial_segments=48).rotate_y(math.pi / 2.0)
    cap.translate(-0.004, 0.0, 0.0)
    for rr in (0.014, 0.022, 0.030):
        ring = TorusGeometry(
            rr, 0.0016, radial_segments=10, tubular_segments=40
        ).rotate_y(math.pi / 2.0)
        ring.translate(-0.011, 0.0, 0.0)
        cap.merge(ring)
    return cap


def _cord_geom():
    """Power cord + strain relief + plug + pins, in handle-local frame."""
    cord_pts = [
        (-0.018, 0.0, -0.080),  # at handle bottom
        (-0.011, 0.0, -0.114),  # drooping down
        (-0.033, 0.0, -0.154),  # further down and back
        (-0.083, 0.0, -0.172),  # continuing back
        (-0.123, 0.0, -0.172),  # leveling out
        (-0.153, 0.0, -0.172),  # at plug
    ]
    cord = tube_from_spline_points(
        cord_pts, radius=0.0042, samples_per_segment=16, radial_segments=14
    )
    # Strain-relief sleeve where cord exits handle base.
    relief = CylinderGeometry(0.0085, 0.024).translate(-0.018, 0.0, -0.082)
    cord.merge(relief)
    # Plug body.
    plug = BoxGeometry((0.040, 0.028, 0.022)).translate(-0.153, 0.0, -0.172)
    cord.merge(plug)
    # Two pins.
    for py in (-0.008, 0.008):
        pin = CylinderGeometry(0.0035, 0.020).rotate_y(math.pi / 2.0)
        pin.translate(-0.182, py, -0.172)
        cord.merge(pin)
    return cord


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hair_dryer_travel")

    shell_pink = model.material("shell_pink", rgba=(0.96, 0.71, 0.78, 1.0))
    dark = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    switch_gray = model.material("switch_gray", rgba=(0.32, 0.32, 0.34, 1.0))

    # ---- BODY (root): barrel + rear filter + hinge barrel + lock tab ----
    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_barrel_solid(), "barrel_shell"),
        material=shell_pink,
        name="barrel_shell",
    )

    body.visual(
        mesh_from_geometry(_rear_filter_geom(), "rear_filter"),
        material=dark,
        name="rear_filter",
    )

    # Visible hinge barrel at barrel underside (cylinder along Y).
    hinge_barrel = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(0.007)
        .extrude(0.016, both=True)  # 32mm along Y
    )
    body.visual(
        mesh_from_cadquery(hinge_barrel, "hinge_barrel"),
        material=dark,
        name="hinge_barrel",
    )

    # Lock tab: small latch button behind the hinge for locked-open detent.
    body.visual(
        Box((0.008, 0.010, 0.004)),
        origin=Origin(xyz=(HINGE_X - 0.015, 0.0, HINGE_Z - 0.002)),
        material=dark,
        name="lock_tab",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.18, 0.085, 0.085)), mass=0.33, origin=Origin(xyz=(0.085, 0.0, 0.0))
    )

    # ---- HANDLE: folding handle with hinge ear + switch housing ----
    handle = model.part("handle")

    handle.visual(
        mesh_from_cadquery(_handle_solid(), "handle_shell"),
        material=shell_pink,
        name="handle_shell",
    )

    # Switch housing plate on the +Y face of the handle.
    handle.visual(
        Box((0.032, 0.005, 0.040)),
        origin=Origin(xyz=(-0.003, 0.0145, -0.040)),
        material=dark,
        name="switch_housing",
    )

    handle.inertial = Inertial.from_geometry(
        Box((0.05, 0.036, 0.08)), mass=0.12, origin=Origin(xyz=(0.0, 0.0, -0.040))
    )

    # Body-to-handle: REVOLUTE hinge at barrel underside.
    # axis=(0,-1,0): right-hand rule about -Y takes handle from -Z (deployed)
    # toward +X (folded alongside barrel toward nozzle).
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.5),
    )

    # ---- NOZZLE: concentrator, spins about barrel axis ----
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

    # ---- SLIDE SWITCHES: power + heat, on handle face ----
    switch_configs = [("power_switch", -0.028), ("heat_switch", -0.052)]
    for i in range(len(switch_configs)):
        name, sz = switch_configs[i]
        sw = model.part(name)
        sw.visual(
            Box((0.013, 0.007, 0.011)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=switch_gray,
            name=f"{name}_nub",
        )
        sw.inertial = Inertial.from_geometry(Box((0.013, 0.007, 0.011)), mass=0.003)
        model.articulation(
            f"handle_to_{name}",
            ArticulationType.PRISMATIC,
            parent=handle,
            child=sw,
            origin=Origin(xyz=(-0.003, 0.0205, sz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.1, lower=-0.007, upper=0.007
            ),
        )

    # ---- POWER CORD + PLUG: drooping cable, fixed to handle base ----
    power_cord = model.part("power_cord")
    power_cord.visual(
        mesh_from_geometry(_cord_geom(), "power_cord"),
        material=dark,
        name="cord_shell",
    )
    power_cord.inertial = Inertial.from_geometry(Box((0.18, 0.03, 0.12)), mass=0.06)
    model.articulation(
        "handle_to_cord",
        ArticulationType.FIXED,
        parent=handle,
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
    handle = object_model.get_part("handle")
    nozzle = object_model.get_part("nozzle")
    power = object_model.get_part("power_switch")
    heat = object_model.get_part("heat_switch")
    cord = object_model.get_part("power_cord")

    hinge = object_model.get_articulation("body_to_handle")
    spin = object_model.get_articulation("barrel_to_nozzle")
    power_joint = object_model.get_articulation("handle_to_power_switch")

    # ---- Folding handle hinge ----
    # Hinge barrel is intentionally nested inside handle hinge ear.
    ctx.allow_overlap(
        body,
        handle,
        elem_a="hinge_barrel",
        elem_b="handle_shell",
        reason="Hinge barrel is intentionally nested inside the handle hinge ear to form the visible revolute hinge.",
    )
    # Handle hinge ear passes through the barrel wall at the connection point.
    ctx.allow_overlap(
        body,
        handle,
        elem_a="barrel_shell",
        elem_b="handle_shell",
        reason="Handle hinge ear passes through the barrel bottom wall at the hinge connection to form a structurally credible folding joint.",
    )

    # Hinge barrel stays within handle ear along Y (ear is wider).
    ctx.expect_within(
        body,
        handle,
        axes="y",
        inner_elem="hinge_barrel",
        outer_elem="handle_shell",
        margin=0.001,
        name="hinge barrel stays within handle ear along Y",
    )

    # Barrel and handle are in contact at the hinge.
    ctx.expect_contact(
        body,
        handle,
        elem_a="barrel_shell",
        elem_b="handle_shell",
        name="barrel and handle contact at hinge",
    )

    # Helper: compute AABB center Z for a part.
    def _aabb_center_z(p):
        aabb = ctx.part_world_aabb(p)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    def _aabb_center_x(p):
        aabb = ctx.part_world_aabb(p)
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    # At q=0 (locked open), handle AABB center is below barrel AABB center.
    handle_cz = _aabb_center_z(handle)
    body_cz = _aabb_center_z(body)
    ctx.check(
        "handle hangs below barrel when deployed",
        handle_cz is not None
        and body_cz is not None
        and handle_cz < body_cz - 0.02,
        details=f"handle_cz={handle_cz}, body_cz={body_cz}",
    )

    # Positive q folds handle toward nozzle (+X direction).
    rest_cx = _aabb_center_x(handle)
    with ctx.pose({hinge: 1.0}):
        folded_cx = _aabb_center_x(handle)
    ctx.check(
        "handle folds toward nozzle",
        rest_cx is not None
        and folded_cx is not None
        and folded_cx > rest_cx + 0.02,
        details=f"rest_cx={rest_cx}, folded_cx={folded_cx}",
    )

    # At max fold, handle rises significantly (swings up alongside barrel).
    with ctx.pose({hinge: 1.5}):
        max_cz = _aabb_center_z(handle)
    ctx.check(
        "handle rises at max fold angle",
        handle_cz is not None
        and max_cz is not None
        and max_cz > handle_cz + 0.03,
        details=f"rest_cz={handle_cz}, max_cz={max_cz}",
    )

    # Hinge articulation is REVOLUTE with proper limits.
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    hinge_limits = hinge.motion_limits
    ctx.check(
        "hinge lower limit is zero (locked-open deployed)",
        hinge_limits is not None
        and hinge_limits.lower is not None
        and abs(hinge_limits.lower) < 1e-6,
        details=f"lower={hinge_limits.lower if hinge_limits else None}",
    )

    # ---- Nozzle (unchanged behavior) ----
    ctx.allow_overlap(
        nozzle,
        body,
        elem_a="nozzle_shell",
        elem_b="barrel_shell",
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

    # Spinning the nozzle reorients the flat slot.
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

    # ---- Switches on handle ----
    ctx.expect_contact(power, handle, name="power switch rests on handle")
    ctx.expect_contact(heat, handle, name="heat switch rests on handle")
    rest_px = ctx.part_world_position(power)[0]
    with ctx.pose({power_joint: 0.007}):
        slid_px = ctx.part_world_position(power)[0]
    ctx.check(
        "power switch slides forward",
        slid_px > rest_px + 0.004,
        details=f"rest_x={rest_px}, slid_x={slid_px}",
    )

    # ---- Cord attached to handle base ----
    ctx.allow_overlap(
        handle,
        cord,
        elem_a="handle_shell",
        elem_b="cord_shell",
        reason="Strain-relief sleeve and cord top intentionally enter the handle base.",
    )
    ctx.expect_contact(cord, handle, name="cord attached to handle")

    return ctx.report()


object_model = build_object_model()
