from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ── Sheave dimensions (preserved from parent) ──────────────────────────
SHEAVE_WIDTH = 0.014
SHEAVE_RADIUS = 0.0185
SHEAVE_BORE_RADIUS = 0.0055
AXLE_RADIUS = 0.0052

# ── Mortise shell dimensions ───────────────────────────────────────────
SHELL_DX = 0.048          # X depth (across rope-plane)
SHELL_DY = 0.034          # Y width (axle direction)
SHELL_DZ = 0.096          # Z height
SHELL_CENTER_Z = 0.016    # body centre offset so sheave sits below mid-height
SHELL_HALF_DY = SHELL_DY / 2.0

# Cavity / rope mouth
CAVITY_RADIUS = 0.021
CAVITY_HALF_Y = 0.014     # pocket ±Y extent inside the shell walls
MOUTH_HALF_X = 0.007      # half-width of rope entry slot
MOUTH_CENTER_Z = -0.015   # slot centre below sheave
MOUTH_HEIGHT_Z = 0.050    # slot total Z extent

# Axle
AXLE_BORE_RADIUS = 0.006
AXLE_LENGTH = 0.038
AXLE_CAP_OFFSET = SHELL_HALF_DY + 0.0012  # cap centre just outside shell face

# Attachment eye + strop groove
EYE_Z = 0.044
EYE_RADIUS = 0.005
STROP_Z = 0.054
STROP_WIDTH = 0.004
STROP_DEPTH = 0.003


# ════════════════════════════════════════════════════════════════════════
# Geometry helpers
# ════════════════════════════════════════════════════════════════════════


def _grooved_sheave_geometry() -> MeshGeometry:
    """Lathe a small rope sheave around the local Y axis with a central bore."""
    profile = [
        (0.0160, -SHEAVE_WIDTH / 2.0),
        (0.0182, -0.0052),
        (0.0188, -0.0032),
        (0.0150, 0.0000),
        (0.0188, 0.0032),
        (0.0182, 0.0052),
        (0.0160, SHEAVE_WIDTH / 2.0),
        (SHEAVE_BORE_RADIUS, SHEAVE_WIDTH / 2.0),
        (SHEAVE_BORE_RADIUS, -SHEAVE_WIDTH / 2.0),
    ]
    segments = 72
    geom = MeshGeometry()
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        for radius, y in profile:
            geom.add_vertex(radius * c, y, radius * s)

    n = len(profile)
    for i in range(segments):
        j = (i + 1) % segments
        for k in range(n):
            a = i * n + k
            b = j * n + k
            c = j * n + ((k + 1) % n)
            d = i * n + ((k + 1) % n)
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _mortise_shell_body():
    """Solid rounded mortise-block shell built with CadQuery.

    Features:
    - Rounded box body (volumetric envelope)
    - Cylindrical sheave pocket along Y
    - Narrow rope-mouth slot opening from the bottom
    - Axle bore through Y
    - Attachment eye hole through X near the top
    - Shallow strop groove around the upper perimeter
    """

    # ── outer body ──────────────────────────────────────────────
    body = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, SHELL_CENTER_Z))
        .box(SHELL_DX, SHELL_DY, SHELL_DZ)
        .edges("|Z")
        .fillet(0.010)
    )
    # small break-edge on top / bottom horizontals
    body = body.edges("#Z").fillet(0.003)

    # ── sheave pocket (cylindrical cavity along Y at origin) ───
    pocket = (
        cq.Workplane("XZ")
        .circle(CAVITY_RADIUS)
        .extrude(CAVITY_HALF_Y, both=True)
    )
    body = body.cut(pocket)

    # ── rope mouth (narrow slot from bottom into pocket) ───────
    mouth = (
        cq.Workplane("XZ")
        .center(0, MOUTH_CENTER_Z)
        .rect(MOUTH_HALF_X * 2, MOUTH_HEIGHT_Z)
        .extrude(CAVITY_HALF_Y, both=True)
    )
    body = body.cut(mouth)

    # ── axle bore (through-hole along Y at origin) ─────────────
    bore = (
        cq.Workplane("XZ")
        .circle(AXLE_BORE_RADIUS)
        .extrude(SHELL_HALF_DY + 0.002, both=True)
    )
    body = body.cut(bore)

    # ── attachment eye (through-hole along X near top) ─────────
    eye = (
        cq.Workplane("YZ")
        .center(0, EYE_Z)
        .circle(EYE_RADIUS)
        .extrude(SHELL_DX / 2.0 + 0.002, both=True)
    )
    body = body.cut(eye)

    # ── strop groove (shallow perimeter channel near top) ──────
    groove_outer = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, STROP_Z))
        .box(SHELL_DX + 0.002, SHELL_DY + 0.002, STROP_WIDTH)
    )
    groove_inner = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, STROP_Z))
        .box(SHELL_DX - 2 * STROP_DEPTH, SHELL_DY - 2 * STROP_DEPTH, STROP_WIDTH + 0.002)
    )
    groove_cutter = groove_outer.cut(groove_inner)
    body = body.cut(groove_cutter)

    return body


# ════════════════════════════════════════════════════════════════════════
# Object model
# ════════════════════════════════════════════════════════════════════════


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="mortise_rope_block",
        meta={
            "run_notes": (
                "Solid mortise block variant: a single rounded shell body fully "
                "encloses the grooved sheave.  The rope mouth slot at the bottom "
                "exposes the sheave groove; a strop groove encircles the upper "
                "perimeter and an attachment eye sits near the top.  Varnished-wood "
                "colorway with stainless-steel axle hardware."
            )
        },
    )

    wood = model.material("varnished_wood", rgba=(0.52, 0.32, 0.12, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("dark_shadow", rgba=(0.03, 0.035, 0.04, 1.0))

    # ── frame (root): mortise shell + axle hardware ────────────────────
    frame = model.part("frame")

    # Main shell body — replaces the two side cheek plates
    frame.visual(
        mesh_from_cadquery(
            _mortise_shell_body(),
            "shell_body",
            tolerance=0.0004,
            angular_tolerance=0.08,
        ),
        material=wood,
        name="shell_body",
    )

    # Axle pin through the shell bore
    frame.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_pin",
    )

    # Axle retaining caps on outside shell faces
    for cap_name, y in (
        ("front_axle_cap", AXLE_CAP_OFFSET),
        ("rear_axle_cap", -AXLE_CAP_OFFSET),
    ):
        frame.visual(
            Cylinder(radius=0.0084, length=0.0024),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=cap_name,
        )

    # ── sheave: grooved wheel captured inside the shell ────────────────
    sheave = model.part("sheave")
    sheave.visual(
        mesh_from_geometry(_grooved_sheave_geometry(), "grooved_sheave_mesh"),
        material=steel,
        name="grooved_sheave",
    )
    # Witness mark makes the continuous spin visible in tests
    sheave.visual(
        Box((0.010, 0.00045, 0.0014)),
        origin=Origin(xyz=(0.010, SHEAVE_WIDTH / 2.0 + 0.00022, 0.0)),
        material=dark,
        name="rotation_mark",
    )

    # ── articulation: sheave spins freely on the axle ──────────────────
    model.articulation(
        "frame_to_sheave",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=sheave,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=20.0),
    )
    return model


# ════════════════════════════════════════════════════════════════════════
# Tests
# ════════════════════════════════════════════════════════════════════════


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sheave = object_model.get_part("sheave")
    spin = object_model.get_articulation("frame_to_sheave")

    # ── shell_body encloses the sheave (variant-specific proof) ────────
    ctx.expect_within(
        sheave,
        frame,
        axes="xz",
        inner_elem="grooved_sheave",
        outer_elem="shell_body",
        margin=0.002,
        name="mortise shell_body encloses the sheave in XZ projection",
    )
    ctx.expect_within(
        sheave,
        frame,
        axes="y",
        inner_elem="grooved_sheave",
        outer_elem="shell_body",
        margin=0.002,
        name="sheave is captured within the shell_body width",
    )

    # Sheave bore surrounds the axle location
    ctx.expect_overlap(
        sheave,
        frame,
        axes="xz",
        elem_a="grooved_sheave",
        elem_b="axle_pin",
        min_overlap=0.009,
        name="sheave bore surrounds the axle location in projection",
    )
    ctx.expect_within(
        sheave,
        frame,
        axes="y",
        inner_elem="grooved_sheave",
        outer_elem="axle_pin",
        margin=0.008,
        name="sheave is retained within the axle span",
    )

    # ── continuous rotation proof ──────────────────────────────────────
    rest_origin = ctx.part_world_position(sheave)
    rest_mark = ctx.part_element_world_aabb(sheave, elem="rotation_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_origin = ctx.part_world_position(sheave)
        turned_mark = ctx.part_element_world_aabb(sheave, elem="rotation_mark")

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_center = _center(rest_mark)
    turned_center = _center(turned_mark)
    ctx.check(
        "continuous sheave spin keeps axle center fixed",
        rest_origin is not None
        and turned_origin is not None
        and all(abs(rest_origin[i] - turned_origin[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_origin}, turned={turned_origin}",
    )
    ctx.check(
        "rotation mark moves around the sheave axis",
        rest_center is not None
        and turned_center is not None
        and abs(rest_center[0] - turned_center[0]) > 0.006
        and abs(rest_center[2] - turned_center[2]) > 0.006,
        details=f"rest_mark={rest_center}, turned_mark={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
