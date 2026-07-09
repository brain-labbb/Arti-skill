from __future__ import annotations

# Squat square-shouldered pump bottle with:
#   - hollow rectangular body with rounded corners and flat (square) shoulder
#   - cylindrical neck rising from center of shoulder
#   - molded volume bands (3 ridges) around the body
#   - tether tab with through-hole connected to the neck
#   - pump head that slides down (prismatic) and rotates slightly (revolute)
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (m) ----
BODY_W = 0.080  # body width (X)
BODY_D = 0.080  # body depth (Y)
BODY_H = 0.065  # body height (Z)
CORNER_R = 0.008  # fillet on vertical edges
WALL = 0.002

NECK_R = 0.013  # neck outer radius
NECK_H = 0.020  # neck height
NECK_BORE_R = 0.009  # neck inner bore
NECK_TOP_Z = BODY_H + NECK_H  # 0.085

# Volume bands
BAND_P = 0.0012  # protrusion beyond body surface
BAND_H = 0.003  # band height
BAND_POSITIONS = [0.015, 0.032, 0.049]  # z of band bottom edges

# Pump head
DISC_R = 0.015
DISC_H = 0.005
STEM_R = 0.004
STEM_H = 0.012
NOZZLE_R = 0.003
NOZZLE_L = 0.020

# Joint limits
PUMP_SLIDE_MAX = 0.008  # max downward travel (m)
PUMP_ROTATE_LIMIT = 0.30  # max rotation (rad, ~17 deg)

# Tether tab
TAB_W = 0.010
TAB_D = 0.006
TAB_H = 0.012
HOLE_R = 0.002


def _body_solid() -> cq.Workplane:
    """Hollow body + neck + tether tab as one CadQuery solid."""
    # Outer body: rounded rectangular box
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(CORNER_R)
    )

    # Neck cylinder rising from shoulder
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    outer = outer.union(neck)

    # Neck rim (slight lip at top for thread finish)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.003)
        .circle(NECK_R + 0.0015)
        .extrude(0.003)
    )
    outer = outer.union(rim)

    # Tether tab: small box extending from neck side with through-hole
    tab_cx = NECK_R + TAB_W / 2.0 - 0.001  # overlap 1mm into neck
    tab_z0 = BODY_H  # tab bottom on shoulder

    tab = (
        cq.Workplane("XY")
        .transformed(offset=(tab_cx, 0.0, tab_z0))
        .box(TAB_W, TAB_D, TAB_H, centered=(True, True, False))
    )

    # Through-hole along Y at tab center
    hole_cz = tab_z0 + TAB_H * 0.5
    hole_cutter = (
        cq.Workplane("XZ")
        .workplane(offset=-(TAB_D / 2.0 + 0.001))
        .center(tab_cx, hole_cz)
        .circle(HOLE_R)
        .extrude(TAB_D + 0.002)
    )
    tab = tab.cut(hole_cutter)
    outer = outer.union(tab)

    # Hollow interior: cut cavity from inside, leaving a ceiling at shoulder
    # so the neck stays connected to the body shell.
    cavity_h = BODY_H - 2 * WALL  # leaves WALL-thick ceiling at top
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(BODY_W - 2 * WALL, BODY_D - 2 * WALL, cavity_h,
             centered=(True, True, False))
        .edges("|Z")
        .fillet(max(0.001, CORNER_R - WALL))
    )

    # Neck bore punches through the ceiling and up through the neck/rim
    bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - WALL - 0.001)
        .circle(NECK_BORE_R)
        .extrude(NECK_H + WALL + 0.010)
    )

    result = outer.cut(inner).cut(bore)

    # Volume bands: thin ridge rings protruding from the body surface.
    # Added after shelling so they sit on the outer wall and remain connected.
    for zb in BAND_POSITIONS:
        band_outer = (
            cq.Workplane("XY")
            .workplane(offset=zb)
            .box(BODY_W + 2 * BAND_P, BODY_D + 2 * BAND_P, BAND_H,
                 centered=(True, True, False))
            .edges("|Z")
            .fillet(CORNER_R + BAND_P)
        )
        # Inner cutter slightly smaller than body so the band overlaps
        # the shell wall by ~0.5mm for guaranteed mesh connectivity.
        band_inner = (
            cq.Workplane("XY")
            .workplane(offset=zb - 0.0001)
            .box(BODY_W - 0.001, BODY_D - 0.001, BAND_H + 0.0002,
                 centered=(True, True, False))
            .edges("|Z")
            .fillet(max(0.001, CORNER_R - 0.001))
        )
        band = band_outer.cut(band_inner)
        result = result.union(band)

    return result


def _pump_solid() -> cq.Workplane:
    """Pump actuator: disc + dome + nozzle + dip-tube stem."""
    disc = (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_H)
    )

    # Slight dome on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=DISC_H)
        .circle(DISC_R * 0.65)
        .extrude(0.002)
    )
    disc = disc.union(dome)

    # Stem going down into the neck
    stem = (
        cq.Workplane("XY")
        .workplane(offset=-STEM_H)
        .circle(STEM_R)
        .extrude(STEM_H + 0.001)
    )
    disc = disc.union(stem)

    # Nozzle: horizontal cylinder pointing +X from the disc
    nozzle_z = DISC_H * 0.5
    nozzle = (
        cq.Workplane("YZ")
        .center(0.0, nozzle_z)
        .circle(NOZZLE_R)
        .extrude(DISC_R + NOZZLE_L)
    )
    disc = disc.union(nozzle)

    # Nozzle tip (slightly wider ring at the end)
    tip = (
        cq.Workplane("YZ")
        .center(0.0, nozzle_z)
        .circle(NOZZLE_R * 1.6)
        .extrude(DISC_R + NOZZLE_L - 0.002)
    )
    # Make the tip a ring by cutting the inner bore
    tip_bore = (
        cq.Workplane("YZ")
        .center(0.0, nozzle_z)
        .circle(NOZZLE_R)
        .extrude(DISC_R + NOZZLE_L + 0.001)
    )
    tip_ring = tip.cut(tip_bore)
    disc = disc.union(tip_ring)

    return disc


def _neck_threads():
    """Thread rings on the neck as a mesh geometry."""
    g = None
    for zt in (BODY_H + 0.005, BODY_H + 0.012):
        ring = TorusGeometry(NECK_R - 0.0006, 0.0010,
                             radial_segments=10, tubular_segments=40)
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_pump_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.74, 0.82, 0.86, 0.28))
    neck_mat = model.material("neck_clear", rgba=(0.70, 0.78, 0.83, 0.35))
    pump_mat = model.material("pump_grey", rgba=(0.55, 0.55, 0.54, 1.0))

    # ---- bottle body (root) ----
    body = model.part("bottle_body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "bottle_shell"),
        material=clear,
        name="bottle_shell",
    )
    body.visual(
        _neck_threads(),
        material=neck_mat,
        name="neck_threads",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, NECK_TOP_Z)),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- pump carrier (massless intermediate frame) ----
    carrier = model.part("pump_carrier")
    carrier.inertial = Inertial.from_geometry(
        Box((0.006, 0.006, 0.006)), mass=1e-4,
    )

    # ---- pump head ----
    pump = model.part("pump_head")
    pump.visual(
        mesh_from_cadquery(_pump_solid(), "pump_body"),
        material=pump_mat,
        name="pump_body",
    )
    pump.inertial = Inertial.from_geometry(
        Cylinder(DISC_R, DISC_H + STEM_H),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (DISC_H - STEM_H) / 2.0)),
    )

    # ---- Articulations ----

    # pump_slide: body -> carrier, PRISMATIC along -Z
    # Positive q presses the pump downward.
    model.articulation(
        "pump_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PUMP_SLIDE_MAX,
            effort=3.0,
            velocity=0.5,
        ),
    )

    # pump_rotate: carrier -> pump_head, REVOLUTE about Z
    # Slight twist range for the pump actuator.
    model.articulation(
        "pump_rotate",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=pump,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PUMP_ROTATE_LIMIT,
            upper=PUMP_ROTATE_LIMIT,
            effort=1.0,
            velocity=1.5,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    pump = object_model.get_part("pump_head")
    carrier = object_model.get_part("pump_carrier")
    slide = object_model.get_articulation("pump_slide")
    rotate = object_model.get_articulation("pump_rotate")

    # --- squat shape: body width comparable to height ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "bottle is squat (width >= 75% of height)",
        body_ext[0] >= body_ext[2] * 0.75,
        details=f"body extents (x,y,z)={body_ext}",
    )

    # --- square shoulders: body is much wider than neck ---
    ctx.check(
        "square shoulders (body much wider than neck)",
        BODY_W > NECK_R * 5,
        details=f"body_w={BODY_W}, neck_r*5={NECK_R * 5}",
    )

    # --- volume bands protrude from body (shell wider than base body) ---
    shell_aabb = ctx.part_element_world_aabb(body, elem=body.get_visual("bottle_shell"))
    shell_width = shell_aabb[1][0] - shell_aabb[0][0]
    ctx.check(
        "volume bands protrude (shell wider than base body)",
        shell_width > BODY_W + 0.001,
        details=f"shell width={shell_width:.4f}, body_w+0.001={BODY_W + 0.001:.4f}",
    )

    # --- tether tab present (body extends past neck on +X side) ---
    body_max_x = ctx.part_world_aabb(body)[1][0]
    ctx.check(
        "tether tab extends past neck on +X",
        body_max_x > NECK_R + 0.004,
        details=f"body max_x={body_max_x:.4f}, expected > {NECK_R + 0.004:.4f}",
    )

    # --- pump head mounted on top ---
    pump_pos = ctx.part_world_position(pump)
    ctx.check(
        "pump head on top of bottle",
        pump_pos is not None and pump_pos[2] > BODY_H,
        details=f"pump origin z={pump_pos}",
    )

    # --- pump_slide: prismatic, positive q moves pump down ---
    ctx.check(
        "pump_slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
    )
    z_rest_top = ctx.part_world_aabb(pump)[1][2]
    with ctx.pose({slide: PUMP_SLIDE_MAX}):
        z_pressed_top = ctx.part_world_aabb(pump)[1][2]
    ctx.check(
        "pump slides down when pressed (positive q)",
        z_pressed_top < z_rest_top - 0.003,
        details=f"rest top z={z_rest_top:.4f}, pressed top z={z_pressed_top:.4f}",
    )

    # --- pump_rotate: revolute with small range ---
    ctx.check(
        "pump_rotate is revolute",
        rotate.articulation_type == ArticulationType.REVOLUTE,
    )
    rlim = rotate.motion_limits
    ctx.check(
        "pump_rotate has bounded small range (< 1 rad)",
        rlim.lower is not None and rlim.upper is not None
        and abs(rlim.upper - rlim.lower) < 1.0,
        details=f"limits=[{rlim.lower}, {rlim.upper}]",
    )

    # Rotation moves the nozzle in XY
    aabb_rest = ctx.part_world_aabb(pump)
    with ctx.pose({rotate: PUMP_ROTATE_LIMIT}):
        aabb_rot = ctx.part_world_aabb(pump)
    xy_shift = (abs(aabb_rot[1][0] - aabb_rest[1][0])
                + abs(aabb_rot[0][0] - aabb_rest[0][0])
                + abs(aabb_rot[1][1] - aabb_rest[1][1])
                + abs(aabb_rot[0][1] - aabb_rest[0][1]))
    ctx.check(
        "pump rotation shifts nozzle in XY",
        xy_shift > 0.002,
        details=f"xy_shift={xy_shift:.4f}",
    )

    # --- overlap allowances: pump stem inside neck ---
    ctx.allow_overlap(
        pump, body,
        elem_a="pump_body",
        elem_b="bottle_shell",
        reason="Pump stem is intentionally inserted into the neck bore.",
    )
    ctx.expect_overlap(
        pump, body,
        axes="z",
        elem_a="pump_body",
        elem_b="bottle_shell",
        min_overlap=0.002,
        name="pump stem overlaps with neck region",
    )

    return ctx.report()


object_model = build_object_model()
