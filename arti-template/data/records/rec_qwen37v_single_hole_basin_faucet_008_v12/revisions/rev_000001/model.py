from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant: squat body on wide oval pedestal,
# pull-up drain rod behind body, lever handle with grip grooves.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Overall height ~0.10 m. Chrome finish throughout.
# ---------------------------------------------------------------------------

# Oval pedestal base plate.
PED_RX = 0.040   # half-extent along X (front-back)
PED_RY = 0.030   # half-extent along Y (left-right)
PED_H = 0.008

# Squat cylindrical body on top of pedestal.
BODY_R = 0.022
BODY_Z0 = PED_H
BODY_Z1 = 0.068
BODY_H = BODY_Z1 - BODY_Z0

# Spout exit height on body.
SPOUT_Z = 0.048

# Handle (lever) on top of body.
HANDLE_Z_BASE = BODY_Z1  # handle sits on top of body
HANDLE_R = 0.016         # handle cylinder radius
HANDLE_H = 0.025         # handle height (the grip cylinder)
HANDLE_LEVER_LEN = 0.040 # lever arm length
HANDLE_LEVER_R = 0.006   # lever arm radius

# Groove dimensions on the handle grip.
GROOVE_DEPTH = 0.0015
GROOVE_WIDTH = 0.002
GROOVE_COUNT = 5
GROOVE_SPACING = 0.004

# Drain rod behind the body.
DRAIN_ROD_R = 0.003
DRAIN_ROD_LEN = 0.090
DRAIN_ROD_X = -0.026   # behind body (toward -X)
DRAIN_ROD_Z_BASE = 0.020  # emerges from body at this height
DRAIN_ROD_TRAVEL = 0.040  # vertical slide travel

# Handle rotation limits (on/off lever).
HANDLE_TURN_LOWER = 0.0
HANDLE_TURN_UPPER = math.radians(90.0)

# Spout geometry.
SPOUT_R = 0.012


def _build_oval_pedestal() -> cq.Workplane:
    """Wide oval pedestal plate in local frame, z-centered."""
    ped = (
        cq.Workplane("XY")
        .ellipse(PED_RX, PED_RY)
        .extrude(PED_H)
    )
    # Soften top edge.
    ped = ped.edges(">Z").fillet(0.001)
    return ped


def _build_body_column() -> cq.Workplane:
    """Squat cylindrical body column above the pedestal."""
    col = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_H)
    )
    # Add a subtle chamfer at the top edge.
    col = col.edges(">Z").chamfer(0.002)
    # Add a thin decorative groove ring around mid-height.
    groove_z = BODY_H * 0.4
    groove = (
        cq.Workplane("XY", origin=(0.0, 0.0, groove_z))
        .circle(BODY_R + 0.001)
        .circle(BODY_R - 0.001)
        .extrude(0.003)
    )
    col = col.cut(groove)
    return col


def _build_spout_shape() -> cq.Workplane:
    """Short curved spout projecting forward (+X) and curving down.
    Built in spout-local frame at SPOUT_Z on body axis; shank along +X."""
    r_out = SPOUT_R
    shank_x0 = 0.008  # seated inside body
    shank_x1 = 0.030
    bend = 0.022
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet rim.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(r_out - 0.001)
        .workplane(offset=-0.008)
        .circle(r_out + 0.004)
        .loft()
    )
    spout = tube.union(flare)

    # Hollow bore at outlet.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.004))
        .circle(r_out + 0.001)
        .workplane(offset=0.014)
        .circle(r_out - 0.003)
        .loft()
    )
    return spout.cut(bore)


def _build_handle_grip() -> cq.Workplane:
    """Lever handle with circumferential grip grooves.
    Local frame: z=0 at handle bottom, +Z up along handle axis.
    The grip cylinder extends upward; lever arm extends along +X from near top."""
    # Main grip cylinder.
    grip = (
        cq.Workplane("XY")
        .circle(HANDLE_R)
        .extrude(HANDLE_H)
    )
    # Dome top.
    dome = (
        cq.Workplane("XY", origin=(0.0, 0.0, HANDLE_H))
        .circle(HANDLE_R)
        .workplane(offset=0.008)
        .circle(0.002)
        .loft()
    )
    grip = grip.union(dome)

    # Cut circumferential grooves on the grip surface.
    for i in range(GROOVE_COUNT):
        z_groove = 0.004 + i * GROOVE_SPACING
        groove_cutter = (
            cq.Workplane("XY", origin=(0.0, 0.0, z_groove))
            .circle(HANDLE_R + 0.001)
            .circle(HANDLE_R - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH)
        )
        grip = grip.cut(groove_cutter)

    # Lever arm extending along +X from near the top of the grip.
    lever_z = HANDLE_H - 0.008
    lever = (
        cq.Workplane("XZ", origin=(0.0, 0.0, lever_z))
        .center(HANDLE_R, 0.0)
        .rect(HANDLE_LEVER_LEN, HANDLE_LEVER_R * 2.0)
        .extrude(HANDLE_LEVER_R * 2.0)
    )
    # Reposition lever: it was extruded along Y from the XZ plane
    # Actually let's build it properly as a cylinder along +X
    lever = (
        cq.Workplane("YZ", origin=(HANDLE_R - 0.002, 0.0, lever_z))
        .circle(HANDLE_LEVER_R)
        .extrude(HANDLE_LEVER_LEN)
    )
    # Round the lever end.
    lever_end = (
        cq.Workplane("XY", origin=(HANDLE_R - 0.002 + HANDLE_LEVER_LEN, 0.0, lever_z))
        .circle(HANDLE_LEVER_R)
        .extrude(0.001)
    )
    grip = grip.union(lever).union(lever_end)

    return grip


def _build_drain_rod() -> cq.Workplane:
    """Thin drain lift rod with a small knob on top.
    Local frame: z=0 at rod bottom, +Z up."""
    rod = (
        cq.Workplane("XY")
        .circle(DRAIN_ROD_R)
        .extrude(DRAIN_ROD_LEN)
    )
    # Small knob/cap on top.
    knob = (
        cq.Workplane("XY", origin=(0.0, 0.0, DRAIN_ROD_LEN))
        .circle(DRAIN_ROD_R * 2.0)
        .extrude(0.006)
    )
    knob = knob.edges(">Z").fillet(0.002)
    rod = rod.union(knob)
    # Small flange at bottom for retention.
    flange = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.002))
        .circle(DRAIN_ROD_R * 1.8)
        .extrude(0.004)
    )
    rod = rod.union(flange)
    return rod


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_v12")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("chrome_satin", rgba=(0.78, 0.80, 0.82, 1.0))

    # ---------------- body (root): oval pedestal + squat column ------------
    body = model.part("body")

    # Oval pedestal plate, z-centered so bottom sits at z=0.
    body.visual(
        mesh_from_cadquery(_build_oval_pedestal(), "pedestal", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="chrome",
        name="oval_pedestal",
    )

    # Squat cylindrical body column on top of pedestal.
    body.visual(
        mesh_from_cadquery(_build_body_column(), "body_column", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0)),
        material="chrome",
        name="body_column",
    )

    # ---------------- spout (fixed): short curved tube ---------------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_Z)),
    )

    # ---------------- handle (revolute on/off lever) ----------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_grip(), "handle_grip", tolerance=0.0003),
        material="chrome_satin",
        name="grip_shell",
    )
    # Revolute joint: handle rotates about the body vertical axis (Z).
    # Positive q swings the lever from the rest position (along +X) toward +Y.
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_Z_BASE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=HANDLE_TURN_LOWER,
            upper=HANDLE_TURN_UPPER,
        ),
    )

    # ---------------- drain rod (prismatic vertical slide) ----------------
    drain_rod = model.part("drain_rod")
    drain_rod.visual(
        mesh_from_cadquery(_build_drain_rod(), "drain_rod", tolerance=0.0003),
        material="chrome",
        name="rod_shaft",
    )
    # Prismatic joint: rod slides vertically. Axis +Z means positive q lifts rod up.
    # The rod emerges from behind the body.
    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain_rod,
        origin=Origin(xyz=(DRAIN_ROD_X, 0.0, DRAIN_ROD_Z_BASE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.1,
            lower=0.0,
            upper=DRAIN_ROD_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    drain_rod = object_model.get_part("drain_rod")
    handle_turn = object_model.get_articulation("handle_turn")
    drain_slide = object_model.get_articulation("drain_slide")

    # Intentional seated insertions.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_column",
        reason="Spout shank is seated inside the solid body casting proxy.",
    )
    ctx.allow_overlap(
        drain_rod,
        body,
        elem_a="rod_shaft",
        elem_b="body_column",
        reason="Drain rod passes through a bore in the body rear wall; lower portion nests inside body proxy.",
    )

    # ---- Variant 12: squat body on wide oval pedestal ----
    pedestal_aabb = ctx.part_element_world_aabb(body, elem="oval_pedestal")
    body_aabb = ctx.part_element_world_aabb(body, elem="body_column")
    ctx.check(
        "oval pedestal is wider than the body column (wide pedestal variant)",
        pedestal_aabb is not None
        and body_aabb is not None
        and (pedestal_aabb[1][1] - pedestal_aabb[0][1]) > (body_aabb[1][1] - body_aabb[0][1]) + 0.010,
        details=f"pedestal={pedestal_aabb}, body={body_aabb}",
    )
    ctx.check(
        "pedestal is oval: front-back extent exceeds left-right extent",
        pedestal_aabb is not None
        and (pedestal_aabb[1][0] - pedestal_aabb[0][0]) > (pedestal_aabb[1][1] - pedestal_aabb[0][1]) + 0.005,
        details=f"pedestal={pedestal_aabb}",
    )
    ctx.check(
        "body is squat: height less than 0.08 m above pedestal",
        body_aabb is not None
        and (body_aabb[1][2] - body_aabb[0][2]) < 0.08,
        details=f"body={body_aabb}",
    )

    # ---- Pedestal sits on deck ----
    ctx.check(
        "pedestal sits flat on the deck",
        pedestal_aabb is not None and abs(pedestal_aabb[0][2]) <= 0.0005,
        details=f"pedestal={pedestal_aabb}",
    )

    # ---- Spout projects forward ----
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from the body",
        spout_aabb is not None and spout_aabb[1][0] > 0.040,
        details=f"spout={spout_aabb}",
    )

    # ---- Drain rod behind the body ----
    rod_aabb = ctx.part_world_aabb(drain_rod)
    ctx.check(
        "drain rod is positioned behind the body (toward -X)",
        rod_aabb is not None and body_aabb is not None
        and (rod_aabb[0][0] + rod_aabb[1][0]) / 2.0 < (body_aabb[0][0] + body_aabb[1][0]) / 2.0 - 0.005,
        details=f"rod={rod_aabb}, body={body_aabb}",
    )

    # ---- Drain rod vertical slide: moves up when pulled ----
    rod_rest_pos = ctx.part_world_position(drain_rod)
    with ctx.pose({drain_slide: DRAIN_ROD_TRAVEL}):
        rod_up_pos = ctx.part_world_position(drain_rod)
    ctx.check(
        "drain rod slides upward when pulled (prismatic joint)",
        rod_rest_pos is not None
        and rod_up_pos is not None
        and rod_up_pos[2] > rod_rest_pos[2] + 0.030,
        details=f"rest={rod_rest_pos}, pulled={rod_up_pos}",
    )

    # ---- Drain rod slide limits ----
    dl = drain_slide.motion_limits
    ctx.check(
        "drain rod travel is 0 to 0.04 m",
        dl is not None
        and dl.lower is not None
        and dl.upper is not None
        and abs(dl.lower) < 1e-9
        and abs(dl.upper - DRAIN_ROD_TRAVEL) < 1e-9,
        details=f"limits={dl}",
    )

    # ---- Handle rotation: lever swings from rest ----
    handle_rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_turn: HANDLE_TURN_UPPER}):
        handle_turned_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle rotates when turned (revolute joint swings the lever)",
        handle_rest_aabb is not None
        and handle_turned_aabb is not None
        and handle_rest_aabb[1][0] > handle_turned_aabb[1][0] + 0.020,
        details=f"rest={handle_rest_aabb}, turned={handle_turned_aabb}",
    )

    # ---- Handle turn limits ----
    hl = handle_turn.motion_limits
    ctx.check(
        "handle turn limits are 0 to 90 degrees",
        hl is not None
        and hl.lower is not None
        and hl.upper is not None
        and abs(hl.lower) < 1e-9
        and abs(hl.upper - HANDLE_TURN_UPPER) < 1e-6,
        details=f"limits={hl}",
    )

    # ---- Handle has grooves (grip shell is not a plain cylinder) ----
    # The grooved grip shell should have a smaller Y-extent at mid-height
    # than a smooth cylinder of the same radius would (grooves cut inward).
    grip_aabb = ctx.part_element_world_aabb(handle, elem="grip_shell")
    ctx.check(
        "handle grip has surface detail (grooved shell differs from smooth cylinder)",
        grip_aabb is not None
        and (grip_aabb[1][1] - grip_aabb[0][1]) < 2.0 * HANDLE_R + 0.001,
        details=f"grip aabb={grip_aabb}",
    )

    # ---- Overall faucet height is compact (~0.10 m) ----
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "overall faucet height is compact (under 0.14 m)",
        handle_aabb is not None and handle_aabb[1][2] < 0.14,
        details=f"handle top={handle_aabb}",
    )

    # ---- Handle sits on top of body ----
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=-0.005,
        max_gap=0.005,
        name="handle base contacts the body top",
    )

    return ctx.report()


object_model = build_object_model()
