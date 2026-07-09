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
# Single-hole basin faucet, modern squared monobloc, ~0.13 m tall, chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# ---------------------------------------------------------------------------

# Base flange (round, sits flat on the deck).
FLANGE_R = 0.028
FLANGE_H = 0.006

# Squared monobloc body column.
BODY_W = 0.040   # width (X)
BODY_D = 0.035   # depth (Y)
BODY_H = 0.085   # height above flange
BODY_Z0 = FLANGE_H
BODY_Z1 = BODY_Z0 + BODY_H

# Spout exit: front face of body, at upper third.
SPOUT_Z = BODY_Z0 + BODY_H * 0.72  # ~0.067 m above deck

# Handle pivot: top center of body.
HANDLE_Z = BODY_Z1

# Drain rod: behind body, thin vertical rod.
DRAIN_ROD_R = 0.0025
DRAIN_ROD_LEN = 0.045
DRAIN_ROD_Y = -BODY_D / 2.0 - DRAIN_ROD_R + 0.001  # rod through body guide, 1mm embed
DRAIN_TRAVEL = 0.025

# Handle lever.
HANDLE_LEN = 0.055
HANDLE_W = 0.012
HANDLE_H = 0.010
HANDLE_LIMIT = math.radians(45.0)

# Aerator: circular disc seated at the spout mouth.
AERATOR_R = 0.011
AERATOR_H = 0.004


def _build_body_shape() -> cq.Workplane:
    """Squared monobloc body with slightly chamfered vertical edges."""
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    )
    # Chamfer the four vertical edges for a modern squared look.
    body = body.edges("|Z").chamfer(0.003)
    # Slight chamfer on top edges.
    body = body.edges(">Z").chamfer(0.001)
    return body


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank from front face, smooth downward
    curve, flared open outlet rim with real hollow bore."""
    r_out = 0.013
    shank_x0 = -0.005  # seated slightly inside the body front face
    shank_x1 = 0.035
    bend = 0.025
    end_x = shank_x1 + bend
    end_z = -bend

    # Sweep path: straight then arc down.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(r_out - 0.001)
        .workplane(offset=-0.009)
        .circle(0.016)
        .loft()
    )
    spout = tube.union(flare)

    # Hollow bore through the outlet mouth.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.008))
        .circle(0.013)
        .workplane(offset=0.020)
        .circle(0.009)
        .loft()
    )
    return spout.cut(bore)


def _build_handle_shape() -> cq.Workplane:
    """Single lever handle: flat bar with rounded ends, pivoting on a short
    cylindrical boss. Handle-local origin at the pivot center, lever extends
    along +X."""
    boss = cq.Workplane("XY").circle(0.008).extrude(0.006)
    lever = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.006))
        .box(HANDLE_LEN, HANDLE_W, HANDLE_H, centered=(False, True, False))
    )
    # Round the far end of the lever.
    lever = lever.edges("|Z and >X").fillet(HANDLE_W / 2.0 - 0.001)
    handle = boss.union(lever)
    handle = handle.edges(">Z").chamfer(0.001)
    return handle


def _build_aerator_shape() -> cq.Workplane:
    """Circular aerator insert: thin disc with a ring of small holes to
    represent the mesh screen."""
    disc = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_H)
    # Central through-hole pattern: cut a ring of small holes.
    hole_r = 0.0015
    ring_r = AERATOR_R * 0.6
    n_holes = 8
    for i in range(n_holes):
        ang = 2.0 * math.pi * i / n_holes
        hx = ring_r * math.cos(ang)
        hy = ring_r * math.sin(ang)
        hole = (
            cq.Workplane("XY", origin=(hx, hy, -0.001))
            .circle(hole_r)
            .extrude(AERATOR_H + 0.002)
        )
        disc = disc.cut(hole)
    # Center hole.
    center_hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(0.003)
        .extrude(AERATOR_H + 0.002)
    )
    disc = disc.cut(center_hole)
    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("aerator_mesh", rgba=(0.35, 0.38, 0.40, 1.0))
    model.material("rubber_dark", rgba=(0.12, 0.12, 0.14, 1.0))

    # ---------------- body (root): flange + squared monobloc column --------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_column", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0)),
        material="chrome",
        name="body_column",
    )
    # Small decorative groove near top of body (separation line before handle).
    body.visual(
        Box((BODY_W + 0.001, BODY_D + 0.001, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z1 - 0.005)),
        material="chrome_dark",
        name="body_groove",
    )

    # ---------------- spout (fixed): hollow tube + flared outlet -----------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout_tube", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(BODY_W / 2.0, 0.0, SPOUT_Z)),
    )

    # ---------------- aerator (fixed): seated disc at spout mouth ----------
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_shape(), "aerator_insert", tolerance=0.0003),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # The aerator sits at the bottom of the spout outlet.
    # Spout end is at (BODY_W/2 + 0.035 + 0.025, 0, SPOUT_Z - 0.025)
    # in body frame. The outlet points down (-Z).
    spout_end_x = BODY_W / 2.0 + 0.060
    spout_end_z = SPOUT_Z - 0.025
    model.articulation(
        "aerator_seat",
        ArticulationType.FIXED,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(0.060, 0.0, -0.025 - AERATOR_H / 2.0)),
    )

    # ---------------- handle (revolute): single lever on top ---------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "handle_lever", tolerance=0.0003),
        material="chrome",
        name="handle_lever",
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=-HANDLE_LIMIT, upper=HANDLE_LIMIT
        ),
    )

    # ---------------- drain rod (prismatic): vertical slide behind body ----
    drain_rod = model.part("drain_rod")
    drain_rod.visual(
        Cylinder(radius=DRAIN_ROD_R, length=DRAIN_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LEN / 2.0)),
        material="chrome",
        name="rod_shaft",
    )
    # Small knob at the top of the rod.
    drain_rod.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LEN + 0.003)),
        material="chrome_brushed",
        name="rod_knob",
    )
    # Joint frame at the top-rear of the body; rod slides vertically.
    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain_rod,
        origin=Origin(xyz=(0.0, DRAIN_ROD_Y, BODY_Z1 - 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.05, lower=0.0, upper=DRAIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    handle = object_model.get_part("handle")
    drain_rod = object_model.get_part("drain_rod")
    handle_pivot = object_model.get_articulation("handle_pivot")
    drain_slide = object_model.get_articulation("drain_slide")

    # Intentional seated insertions.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_column",
        reason="Spout shank is intentionally seated ~5 mm into the solid body front face.",
    )
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_disc",
        elem_b="spout_tube",
        reason="Aerator insert is seated inside the flared spout outlet mouth.",
    )
    ctx.allow_overlap(
        drain_rod,
        body,
        elem_a="rod_shaft",
        elem_b="body_column",
        reason="Drain rod passes through a guide bore in the body back face.",
    )

    # ---- body: squared monobloc on round flange ---------------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    column_aabb = ctx.part_element_world_aabb(body, elem="body_column")
    ctx.check(
        "body column is squared (width and depth differ from height)",
        column_aabb is not None
        and abs((column_aabb[1][0] - column_aabb[0][0]) - BODY_W) < 0.005
        and abs((column_aabb[1][1] - column_aabb[0][1]) - BODY_D) < 0.005,
        details=f"column aabb={column_aabb}",
    )

    # ---- spout: projects forward from body front face ---------------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward from the body",
        spout_aabb is not None and spout_aabb[1][0] > BODY_W / 2.0 + 0.040,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout curves down to a low outlet above the deck",
        spout_aabb is not None
        and spout_aabb[0][2] < BODY_Z0 + BODY_H * 0.60
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- aerator: separate insert at spout mouth --------------------------
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator insert sits near the spout outlet",
        aerator_aabb is not None
        and aerator_aabb[0][2] < SPOUT_Z - 0.020
        and aerator_aabb[1][0] > BODY_W / 2.0 + 0.040,
        details=f"aerator aabb={aerator_aabb}",
    )
    ctx.expect_overlap(
        aerator,
        spout,
        axes="xy",
        min_overlap=0.005,
        name="aerator overlaps spout footprint in XY (seated inside)",
    )

    # ---- handle: lever on top, revolute pivot -----------------------------
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle lever sits on top of the body",
        handle_aabb is not None and handle_aabb[0][2] >= BODY_Z1 - 0.002,
        details=f"handle aabb={handle_aabb}",
    )
    hl = handle_pivot.motion_limits
    ctx.check(
        "handle pivot has non-trivial revolute limits",
        hl is not None
        and hl.lower is not None
        and hl.upper is not None
        and hl.lower < 0.0
        and hl.upper > 0.0
        and abs(hl.upper - hl.lower) > math.radians(30.0),
        details=f"limits={hl}",
    )

    # ---- drain rod: behind body, prismatic slide --------------------------
    rod_aabb = ctx.part_world_aabb(drain_rod)
    ctx.check(
        "drain rod is behind the body (negative Y from body center)",
        rod_aabb is not None
        and (rod_aabb[0][1] + rod_aabb[1][1]) / 2.0 < -BODY_D / 2.0 + 0.001,
        details=f"rod aabb={rod_aabb}",
    )
    dl = drain_slide.motion_limits
    ctx.check(
        "drain rod has prismatic travel limits 0 to 25 mm",
        dl is not None
        and dl.lower is not None
        and dl.upper is not None
        and abs(dl.lower) < 1e-9
        and abs(dl.upper - DRAIN_TRAVEL) < 1e-9,
        details=f"limits={dl}",
    )

    # ---- decisive poses: handle rotates, drain rod slides up --------------
    rest_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_lever")
    with ctx.pose({handle_pivot: HANDLE_LIMIT}):
        turned_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_lever")
    ctx.check(
        "handle rotates around the body axis when posed",
        rest_handle_aabb is not None
        and turned_handle_aabb is not None
        and abs((turned_handle_aabb[0][1] + turned_handle_aabb[1][1]) / 2.0
                - (rest_handle_aabb[0][1] + rest_handle_aabb[1][1]) / 2.0) > 0.005,
        details=f"rest={rest_handle_aabb}, turned={turned_handle_aabb}",
    )

    rest_rod = ctx.part_world_position(drain_rod)
    with ctx.pose({drain_slide: DRAIN_TRAVEL}):
        pulled_rod = ctx.part_world_position(drain_rod)
    ctx.check(
        "drain rod slides upward when pulled",
        rest_rod is not None
        and pulled_rod is not None
        and 0.020 <= (pulled_rod[2] - rest_rod[2]) <= 0.030,
        details=f"rest={rest_rod}, pulled={pulled_rod}",
    )

    # ---- overall height check --------------------------------------------
    ctx.check(
        "overall faucet height is about 0.13 m",
        handle_aabb is not None and 0.10 <= handle_aabb[1][2] <= 0.145,
        details=f"handle aabb={handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
