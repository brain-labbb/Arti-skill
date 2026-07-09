from __future__ import annotations

import math

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
    TireGeometry,
    TireSidewall,
    TireTread,
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

HANDLE_X = 0.12       # pull-handle tube spacing from center
TUBE_R = 0.011
TUBE_BOT = 0.12       # retracted tube bottom z
TUBE_TOP = 0.60       # retracted tube top z
HANDLE_TRAVEL = 0.34  # telescoping extension

# Spinner caster wheel dimensions
WHEEL_R = 0.030       # 60 mm diameter spinner wheel
WHEEL_T = 0.022       # wheel width
WHEEL_X = 0.13        # caster X offset from center
WHEEL_Y = 0.072       # caster Y offset from center
WHEEL_CZ = WHEEL_R    # wheel center z (bottom touches ground at z=0)

# Caster hardware dimensions
CASTER_PLATE_R = 0.024
CASTER_PLATE_H = 0.005
FORK_ARM_W = 0.006
FORK_ARM_D = 0.016
FORK_INNER_OFFSET = WHEEL_T / 2.0 + 0.003  # half wheel width + clearance
FORK_TOP_Z = SHELL_Z0 - CASTER_PLATE_H     # 0.045
FORK_ARM_H = FORK_TOP_Z - (WHEEL_CZ - 0.008)  # extends 8 mm below axle
FORK_CZ = FORK_TOP_Z - FORK_ARM_H / 2.0

CASTER_POSITIONS = [
    (-WHEEL_X, -WHEEL_Y),   # 0: front-left
    ( WHEEL_X, -WHEEL_Y),   # 1: front-right
    (-WHEEL_X,  WHEEL_Y),   # 2: rear-left
    ( WHEEL_X,  WHEEL_Y),   # 3: rear-right
]


def _add_spinner_caster(model, body, i, sx, sy, *, wheel_mat, metal_mat, trim_mat):
    """Add one swivel spinner caster at position (sx, sy) on the body underside.

    Body-mounted hardware: swivel raceway plate + fork arms + fork bridge.
    Separate wheel part with tire, hub, axle, tread marker, and CONTINUOUS spin joint.
    """
    # --- Body-mounted caster hardware ---
    # Swivel raceway plate (circular disc against shell underside)
    body.visual(
        Cylinder(radius=CASTER_PLATE_R, length=CASTER_PLATE_H),
        origin=Origin(xyz=(sx, sy, SHELL_Z0 - CASTER_PLATE_H / 2.0)),
        material=metal_mat,
        name=f"swivel_plate_{i}",
    )

    # Fork bridge plate (connects fork arms at top)
    bridge_w = 2.0 * (FORK_INNER_OFFSET + FORK_ARM_W)
    body.visual(
        Box((bridge_w, FORK_ARM_D, 0.005)),
        origin=Origin(xyz=(sx, sy, FORK_TOP_Z - 0.0025)),
        material=trim_mat,
        name=f"fork_bridge_{i}",
    )

    # Fork arms (two plates straddling the wheel along X)
    for side_idx, side in enumerate((-1, 1)):
        arm_x = sx + side * (FORK_INNER_OFFSET + FORK_ARM_W / 2.0)
        body.visual(
            Box((FORK_ARM_W, FORK_ARM_D, FORK_ARM_H)),
            origin=Origin(xyz=(arm_x, sy, FORK_CZ)),
            material=trim_mat,
            name=f"fork_{i}_{side_idx}",
        )

    # --- Wheel part ---
    wheel = model.part(f"wheel_{i}")

    # Tire (disc in YZ plane, width along X — native TireGeometry orientation)
    tire_geo = TireGeometry(
        WHEEL_R,
        WHEEL_T,
        inner_radius=WHEEL_R * 0.55,
        tread=TireTread(style="block", depth=0.003, count=10, land_ratio=0.55),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
    )
    wheel.visual(
        mesh_from_geometry(tire_geo, f"tire_{i}"),
        material=wheel_mat,
        name="tire",
    )

    # Hub disc (metal center cap, sized to clear the fork bridge above)
    wheel.visual(
        Cylinder(radius=0.009, length=WHEEL_T - 0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hub",
    )

    # Axle stub (along X, captured by the fork arms)
    axle_len = 2.0 * (FORK_INNER_OFFSET + FORK_ARM_W) + 0.004
    wheel.visual(
        Cylinder(radius=0.004, length=axle_len),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="axle",
    )

    # Tread marker (small block on tire surface for spin verification)
    wheel.visual(
        Box((0.008, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_R - 0.003)),
        material=metal_mat,
        name="tread_marker",
    )

    # Continuous spin joint: rotates about X (the axle direction)
    model.articulation(
        f"wheel_{i}_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(sx, sy, WHEEL_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_luggage_bag")

    shell_mat = model.material("shell_green", rgba=(0.20, 0.40, 0.16, 1.0))
    trim_mat = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_mat = model.material("metal_chrome", rgba=(0.60, 0.60, 0.63, 1.0))
    wheel_mat = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))

    # --- Body shell (root) ---------------------------------------------------
    body = model.part("luggage_body")
    shell_geo = ExtrudeGeometry.from_z0(
        rounded_rect_profile(W, D, CORNER_R, corner_segments=8),
        SHELL_H,
        cap=True,
    )
    shell_geo.translate(0.0, 0.0, SHELL_Z0)
    body.visual(mesh_from_geometry(shell_geo, "shell"), material=shell_mat, name="shell")

    # central frame seam (darker band around the middle split line)
    for sx in (-1.0, 1.0):
        body.visual(
            Box((0.012, D + 0.006, 0.018)),
            origin=Origin(xyz=(sx * (W / 2.0 - 0.004), 0.0, SHELL_Z0 + SHELL_H * 0.5)),
            material=trim_mat,
            name=f"seam_side_{'l' if sx < 0 else 'r'}",
        )
    body.visual(
        Box((W + 0.006, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -(D / 2.0 - 0.004), SHELL_Z0 + SHELL_H * 0.5)),
        material=trim_mat,
        name="seam_front",
    )

    # top handle housing plate with two exit bosses
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

    # side grab handle on the +X face: two posts + a vertical grip bar
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

    # --- Four swivel spinner casters (for-i-in-range loop) -------------------
    for i in range(4):
        sx, sy = CASTER_POSITIONS[i]
        _add_spinner_caster(
            model, body, i, sx, sy,
            wheel_mat=wheel_mat, metal_mat=metal_mat, trim_mat=trim_mat,
        )

    # --- Telescoping pull handle (prismatic, retracts into the shell) --------
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
    # top crossbar grip joining the two tubes
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

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("luggage_body")
    handle = object_model.get_part("pull_handle")
    handle_joint = object_model.get_articulation("handle_extend")
    wheel0 = object_model.get_part("wheel_0")
    wheel0_spin = object_model.get_articulation("wheel_0_spin")

    # The telescoping handle is an intentional nested fit inside the shell proxy.
    ctx.allow_overlap(
        handle, body,
        elem_a="tube_left", elem_b="shell",
        reason="Left pull-handle tube retracts into the solid shell proxy.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="tube_right", elem_b="shell",
        reason="Right pull-handle tube retracts into the solid shell proxy.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="tube_left", elem_b="handle_boss_l",
        reason="Tube passes through its top exit boss.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="tube_right", elem_b="handle_boss_r",
        reason="Tube passes through its top exit boss.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="tube_left", elem_b="handle_housing",
        reason="Tube passes through the top handle housing plate.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="tube_right", elem_b="handle_housing",
        reason="Tube passes through the top handle housing plate.",
    )

    # Each spinner caster wheel: axle captured by fork, tire recessed into shell well.
    for i in range(4):
        wi = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            wi, body,
            elem_a="axle", elem_b=f"fork_{i}_0",
            reason="Caster axle stub is captured inside the left fork arm.",
        )
        ctx.allow_overlap(
            wi, body,
            elem_a="axle", elem_b=f"fork_{i}_1",
            reason="Caster axle stub is captured inside the right fork arm.",
        )
        ctx.allow_overlap(
            wi, body,
            elem_a="tire", elem_b="shell",
            reason="Spinner wheel is recessed into the shell underside well.",
        )
        ctx.allow_overlap(
            wi, body,
            elem_a="tire", elem_b=f"swivel_plate_{i}",
            reason="Spinner wheel top protrudes into the swivel raceway plate recess.",
        )
        ctx.allow_overlap(
            wi, body,
            elem_a="tire", elem_b=f"fork_bridge_{i}",
            reason="Spinner wheel top protrudes into the fork bridge plate recess.",
        )

    # Retracted handle stays inserted in the shell.
    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_within(
            handle, body,
            axes="xy",
            inner_elem="tube_left", outer_elem="shell",
            margin=0.005,
            name="retracted handle tube stays within the shell footprint",
        )
        ctx.expect_overlap(
            handle, body,
            axes="z",
            elem_a="tube_left", elem_b="shell",
            min_overlap=0.20,
            name="retracted handle is deeply inserted in the shell",
        )
        rest_top = ctx.part_element_world_aabb(handle, elem="cross_grip")

    # Extended handle rises but stays inserted.
    with ctx.pose({handle_joint: HANDLE_TRAVEL}):
        ext_top = ctx.part_element_world_aabb(handle, elem="cross_grip")
        ctx.expect_overlap(
            handle, body,
            axes="z",
            elem_a="tube_left", elem_b="shell",
            min_overlap=0.08,
            name="extended handle retains insertion in the shell",
        )
    ctx.check(
        "pull handle extends upward",
        rest_top is not None and ext_top is not None and ext_top[1][2] > rest_top[1][2] + 0.25,
        details=f"rest_top={rest_top}, ext_top={ext_top}",
    )

    # --- Spinner caster checks ---

    # Four caster wheels exist.
    ctx.check(
        "four spinner caster wheels are present",
        all(object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
        details="expected wheel_0..wheel_3",
    )

    # Each caster has a visible swivel plate on the body underside.
    for i in range(4):
        sv = body.get_visual(f"swivel_plate_{i}")
        ctx.check(
            f"caster_{i} has swivel plate",
            sv is not None,
            details=f"missing swivel_plate_{i} on body",
        )

    # Each caster has fork arms on the body.
    for i in range(4):
        f0 = body.get_visual(f"fork_{i}_0")
        f1 = body.get_visual(f"fork_{i}_1")
        ctx.check(
            f"caster_{i} has fork arms",
            f0 is not None and f1 is not None,
            details=f"missing fork arms for caster {i}",
        )

    # Wheel 0 axle is seated in its fork (contact check).
    ctx.expect_contact(
        wheel0, body,
        elem_a="axle", elem_b="fork_0_0",
        contact_tol=1e-3,
        name="caster 0 axle is seated in its fork arm",
    )

    # Wheel spin: the tread marker sweeps from top toward the rolling direction.
    with ctx.pose({wheel0_spin: 0.0}):
        lug_rest = ctx.part_element_world_aabb(wheel0, elem="tread_marker")
    with ctx.pose({wheel0_spin: math.pi / 2.0}):
        lug_spun = ctx.part_element_world_aabb(wheel0, elem="tread_marker")
    ctx.check(
        "caster wheel spins about its axle",
        lug_rest is not None
        and lug_spun is not None
        and lug_spun[1][2] < lug_rest[1][2] - 0.015,
        details=f"rest={lug_rest}, spun={lug_spun}",
    )

    # Each wheel has its own CONTINUOUS spin articulation.
    for i in range(4):
        j = object_model.get_articulation(f"wheel_{i}_spin")
        ctx.check(
            f"wheel_{i}_spin is CONTINUOUS",
            j.articulation_type == ArticulationType.CONTINUOUS,
            details=f"expected CONTINUOUS, got {j.articulation_type}",
        )

    return ctx.report()
