from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.36          # width (X)
D = 0.22          # depth (Y)
SHELL_Z0 = 0.05   # shell base above the wheels
SHELL_H = 0.52    # shell height
SHELL_TOP = SHELL_Z0 + SHELL_H  # 0.57
CORNER_R = 0.045
WALL_T = 0.003    # shell wall thickness

HANDLE_X = 0.12       # pull-handle tube spacing from center
TUBE_R = 0.011
TUBE_BOT = 0.12       # retracted tube bottom z
TUBE_TOP = 0.60       # retracted tube top z
HANDLE_TRAVEL = 0.34  # telescoping extension

WHEEL_R = 0.035
WHEEL_T = 0.024
WHEEL_X = 0.13
WHEEL_Y = 0.072
WHEEL_CZ = WHEEL_R  # wheel touches ground at z=0

CLEARANCE = 0.001  # fit clearance for flap in opening
FLAP_W = W - 2 * WALL_T - 2 * CLEARANCE
FLAP_H = SHELL_H - 2 * WALL_T - 2 * CLEARANCE
FLAP_T = WALL_T
FLAP_FILLET_R = CORNER_R - WALL_T - CLEARANCE

HINGE_Z = SHELL_TOP - WALL_T  # hinge at top inner edge of opening
HINGE_Y = -D / 2.0            # hinge at front face


def _build_hollow_shell() -> cq.Workplane:
    """Build the hollow shell body: outer rounded box minus inner cavity.

    The front face (-Y) is completely open, revealing the hollow interior
    with visible wall thickness on the side, top, bottom, and back walls.
    """
    # Outer box with filleted vertical edges
    outer = (
        cq.Workplane("XY")
        .box(W, D, SHELL_H)
        .edges("|Z").fillet(CORNER_R)
        .translate((0.0, 0.0, SHELL_Z0 + SHELL_H / 2.0))
    )

    # Inner cavity: slightly over-extends past the front face for clean boolean
    overextend = 0.005
    inner_w = W - 2 * WALL_T
    inner_d = D - WALL_T + overextend
    inner_h = SHELL_H - 2 * WALL_T
    inner_fillet = CORNER_R - WALL_T

    inner = (
        cq.Workplane("XY")
        .box(inner_w, inner_d, inner_h)
        .edges("|Z").fillet(inner_fillet)
        .translate((
            0.0,
            -(overextend + WALL_T) / 2.0,
            SHELL_Z0 + SHELL_H / 2.0,
        ))
    )
    return outer.cut(inner)


def _build_flap_panel():
    """Build the front flap panel in its local frame (origin at hinge point).

    Uses rounded_rect_profile in XY, extrudes for thickness, then rotates
    so the rounded rect lies in XZ and thickness is along Y.
    """
    profile = rounded_rect_profile(FLAP_W, FLAP_H, FLAP_FILLET_R, corner_segments=8)
    geo = ExtrudeGeometry(profile, FLAP_T, cap=True, center=True)
    # Rotate so rounded-rect is in XZ plane, thickness in Y
    geo.rotate_x(-math.pi / 2.0)
    # Shift so hinge (top-center-outer) is at origin
    geo.translate(0.0, FLAP_T / 2.0, -FLAP_H / 2.0)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_luggage_bag")

    shell_mat = model.material("shell_green", rgba=(0.20, 0.40, 0.16, 1.0))
    trim_mat = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_mat = model.material("metal_chrome", rgba=(0.60, 0.60, 0.63, 1.0))
    wheel_mat = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    interior_mat = model.material("interior_gray", rgba=(0.38, 0.36, 0.34, 1.0))

    # ── Body shell (root) ─ hollow thin-walled container, open at front ──
    body = model.part("luggage_body")

    # CadQuery hollow shell
    shell_cq = _build_hollow_shell()
    body.visual(
        mesh_from_cadquery(shell_cq, "shell", tolerance=0.0008),
        material=shell_mat,
        name="shell",
    )

    # Packing floor inside the hollow compartment
    floor_w = W - 2 * WALL_T - 0.006
    floor_d = D - WALL_T - 0.006
    body.visual(
        Box((floor_w, floor_d, 0.004)),
        origin=Origin(xyz=(0.0, -(WALL_T) / 2.0, SHELL_Z0 + WALL_T + 0.002)),
        material=interior_mat,
        name="packing_floor",
    )

    # Side seam strips (structural reinforcement ridges)
    for sx in (-1.0, 1.0):
        body.visual(
            Box((0.012, D + 0.006, 0.018)),
            origin=Origin(xyz=(sx * (W / 2.0 - 0.004), 0.0, SHELL_Z0 + SHELL_H * 0.5)),
            material=trim_mat,
            name=f"seam_side_{'l' if sx < 0 else 'r'}",
        )

    # Top handle housing plate with two exit bosses
    body.visual(
        Box((0.30, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_TOP - 0.004)),
        material=trim_mat,
        name="handle_housing",
    )
    for sx in (-HANDLE_X, HANDLE_X):
        body.visual(
            Cylinder(radius=0.017, length=0.03),
            origin=Origin(xyz=(sx, 0.0, SHELL_TOP)),
            material=trim_mat,
            name=f"handle_boss_{'l' if sx < 0 else 'r'}",
        )

    # Side grab handle on the +X face: two posts + a vertical grip bar
    for sy in (-0.05, 0.05):
        body.visual(
            Box((0.035, 0.022, 0.022)),
            origin=Origin(xyz=(W / 2.0 + 0.0125, sy, SHELL_Z0 + SHELL_H * 0.78)),
            material=trim_mat,
            name=f"side_post_{'f' if sy < 0 else 'b'}",
        )
    body.visual(
        Cylinder(radius=0.011, length=0.12),
        origin=Origin(
            xyz=(W / 2.0 + 0.028, 0.0, SHELL_Z0 + SHELL_H * 0.78),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_mat,
        name="side_grip",
    )

    # Caster yokes that carry the four base wheels
    for i, (sx, sy) in enumerate(
        [(-WHEEL_X, -WHEEL_Y), (WHEEL_X, -WHEEL_Y), (-WHEEL_X, WHEEL_Y), (WHEEL_X, WHEEL_Y)]
    ):
        body.visual(
            Box((0.05, WHEEL_T + 0.018, 0.045)),
            origin=Origin(xyz=(sx, sy, SHELL_Z0 + 0.012)),
            material=trim_mat,
            name=f"yoke_{i}",
        )

    # ── Front access flap (REVOLUTE hinge at top of opening) ─────────────
    flap = model.part("front_flap")
    flap_geo = _build_flap_panel()
    flap.visual(
        mesh_from_geometry(flap_geo, "flap_panel"),
        material=shell_mat,
        name="flap_panel",
    )
    # Latch tab at the bottom of the flap (protrudes outward for grip)
    flap.visual(
        Box((0.045, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.007, -FLAP_H + 0.022)),
        material=trim_mat,
        name="flap_latch",
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=1.4),
    )

    # ── Telescoping pull handle (prismatic, retracts into the shell) ─────
    handle = model.part("pull_handle")
    tube_len = TUBE_TOP - TUBE_BOT
    tube_cz = (TUBE_TOP + TUBE_BOT) / 2.0
    for sx, nm in ((-HANDLE_X, "tube_left"), (HANDLE_X, "tube_right")):
        handle.visual(
            Cylinder(radius=TUBE_R, length=tube_len),
            origin=Origin(xyz=(sx, 0.0, tube_cz)),
            material=metal_mat,
            name=nm,
        )
    handle.visual(
        Cylinder(radius=0.013, length=2.0 * HANDLE_X + 0.02),
        origin=Origin(xyz=(0.0, 0.0, TUBE_TOP), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="cross_grip",
    )
    handle.visual(
        Box((0.10, 0.03, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, TUBE_TOP)),
        material=trim_mat,
        name="grip_pad",
    )

    model.articulation(
        "handle_extend",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.3, lower=0.0, upper=HANDLE_TRAVEL),
    )

    # ── Four spinner wheels (continuous spin about local Y) ──────────────
    for i, (sx, sy) in enumerate(
        [(-WHEEL_X, -WHEEL_Y), (WHEEL_X, -WHEEL_Y), (-WHEEL_X, WHEEL_Y), (WHEEL_X, WHEEL_Y)]
    ):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_T),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.006, length=WHEEL_T + 0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name="axle",
        )
        wheel.visual(
            Box((0.010, WHEEL_T - 0.004, 0.010)),
            origin=Origin(xyz=(WHEEL_R - 0.005, 0.0, 0.0)),
            material=metal_mat,
            name="tread_lug",
        )
        model.articulation(
            f"wheel_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(sx, sy, WHEEL_CZ)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("luggage_body")
    flap = object_model.get_part("front_flap")
    handle = object_model.get_part("pull_handle")
    flap_joint = object_model.get_articulation("body_to_flap")
    handle_joint = object_model.get_articulation("handle_extend")
    wheel0 = object_model.get_part("wheel_0")
    wheel0_spin = object_model.get_articulation("wheel_0_spin")

    # ── Overlap allowances ───────────────────────────────────────────────
    # Handle tubes pass through the hollow shell top wall
    for tube_name, boss_name in (
        ("tube_left", "handle_boss_l"),
        ("tube_right", "handle_boss_r"),
    ):
        ctx.allow_overlap(
            handle, body,
            elem_a=tube_name, elem_b="shell",
            reason=f"{tube_name} passes through the hollow shell top wall.",
        )
        ctx.allow_overlap(
            handle, body,
            elem_a=tube_name, elem_b=boss_name,
            reason=f"{tube_name} passes through its exit boss.",
        )
        ctx.allow_overlap(
            handle, body,
            elem_a=tube_name, elem_b="handle_housing",
            reason=f"{tube_name} passes through the top handle housing plate.",
        )

    # Each wheel is recessed into its caster yoke/housing well
    for i in range(4):
        wi = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            wi, body,
            elem_a="axle", elem_b=f"yoke_{i}",
            reason="Wheel axle stub is captured inside the caster yoke.",
        )
        ctx.allow_overlap(
            wi, body,
            elem_a="tire", elem_b=f"yoke_{i}",
            reason="Spinner wheel is recessed up into its caster housing well.",
        )
        ctx.allow_overlap(
            wi, body,
            elem_a="tire", elem_b="shell",
            reason="Spinner wheel is recessed into the shell underside.",
        )

    # ── Flap mechanism tests ─────────────────────────────────────────────
    # Flap opens outward: the latch (bottom of flap) swings toward -Y
    with ctx.pose({flap_joint: 0.0}):
        flap_closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_latch")
    with ctx.pose({flap_joint: 1.2}):
        flap_open_aabb = ctx.part_element_world_aabb(flap, elem="flap_latch")

    ctx.check(
        "flap opens outward from the front face",
        (flap_closed_aabb is not None
         and flap_open_aabb is not None
         and flap_open_aabb[0][1] < flap_closed_aabb[0][1] - 0.05),
        details=f"closed_latch_aabb={flap_closed_aabb}, open_latch_aabb={flap_open_aabb}",
    )

    # Flap panel is within the opening footprint when closed
    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_within(
            flap, body,
            axes="x",
            inner_elem="flap_panel",
            outer_elem="shell",
            margin=0.005,
            name="closed flap stays within shell width",
        )

    # Flap latch is accessible from outside when closed
    ctx.check(
        "flap latch protrudes outside the shell front face",
        flap_closed_aabb is not None and flap_closed_aabb[0][1] < -D / 2.0 + 0.001,
        details=f"flap_closed_aabb={flap_closed_aabb}",
    )

    # ── Handle tests (same logic as parent) ──────────────────────────────
    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_within(
            handle, body,
            axes="xy",
            inner_elem="tube_left",
            outer_elem="shell",
            margin=0.005,
            name="retracted handle tube stays within the shell footprint",
        )
        ctx.expect_overlap(
            handle, body,
            axes="z",
            elem_a="tube_left",
            elem_b="shell",
            min_overlap=0.01,
            name="retracted handle passes through shell top wall",
        )
        rest_top = ctx.part_element_world_aabb(handle, elem="cross_grip")

    with ctx.pose({handle_joint: HANDLE_TRAVEL}):
        ext_top = ctx.part_element_world_aabb(handle, elem="cross_grip")
        ctx.expect_overlap(
            handle, body,
            axes="z",
            elem_a="tube_left",
            elem_b="shell",
            min_overlap=0.005,
            name="extended handle retains insertion through shell top wall",
        )
    ctx.check(
        "pull handle extends upward",
        rest_top is not None and ext_top is not None and ext_top[1][2] > rest_top[1][2] + 0.25,
        details=f"rest_top={rest_top}, ext_top={ext_top}",
    )

    # ── Wheel tests ──────────────────────────────────────────────────────
    ctx.expect_contact(
        wheel0, body,
        elem_a="axle", elem_b="yoke_0",
        contact_tol=1e-4,
        name="front wheel axle is seated in its yoke",
    )

    with ctx.pose({wheel0_spin: 0.0}):
        lug_rest = ctx.part_element_world_aabb(wheel0, elem="tread_lug")
    with ctx.pose({wheel0_spin: math.pi / 2.0}):
        lug_spun = ctx.part_element_world_aabb(wheel0, elem="tread_lug")
    ctx.check(
        "wheel spins about its axle",
        lug_rest is not None
        and lug_spun is not None
        and lug_spun[1][2] < lug_rest[1][2] - 0.015,
        details=f"rest={lug_rest}, spun={lug_spun}",
    )

    ctx.check(
        "four base wheels are present",
        all(object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
        details="expected wheel_0..wheel_3",
    )

    # ── Hollow interior tests ────────────────────────────────────────────
    # Packing floor is inside the body cavity
    ctx.check(
        "packing floor visual exists on luggage body",
        any(v.name == "packing_floor" for v in body.visuals),
        details="expected packing_floor visual on luggage_body",
    )

    return ctx.report()
