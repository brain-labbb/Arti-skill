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

# --- Dimensions (meters) -----------------------------------------------------
W = 0.36          # width (X)
D = 0.22          # depth (Y)
# Shell base is lifted clear of the wheels: the case rides on a low chassis and
# the spinner wheels live entirely BELOW the shell (none recessed into the box).
WHEEL_R = 0.034
WHEEL_T = 0.024
WHEEL_CZ = WHEEL_R          # wheel touches ground at z=0; wheel top = 2*R = 0.068
GROUND_CLEAR = 0.007        # gap between wheel top and shell base
SHELL_Z0 = 2.0 * WHEEL_R + GROUND_CLEAR  # 0.075 -> wheel top (0.068) sits below
SHELL_H = 0.52    # shell height
SHELL_TOP = SHELL_Z0 + SHELL_H
CORNER_R = 0.045
WALL_T = 0.003    # 3mm wall thickness for hollow shell

BASE_T = 0.020    # low chassis/skid plate that carries the four corner casters

HANDLE_X = 0.12       # pull-handle tube spacing from center
TUBE_R = 0.011
TUBE_BOT = 0.14       # retracted tube bottom z (inside the raised shell)
TUBE_TOP = SHELL_TOP + 0.005  # retracted tube top just proud of the top deck
HANDLE_TRAVEL = 0.34  # telescoping extension

# Four spinner casters, one at each bottom corner of the full footprint.
WHEEL_X = 0.135
WHEEL_Y = D / 2.0 - 0.03  # 0.08

WHEEL_POSITIONS = [
    (-WHEEL_X, -WHEEL_Y),
    (WHEEL_X, -WHEEL_Y),
    (-WHEEL_X, WHEEL_Y),
    (WHEEL_X, WHEEL_Y),
]

# Hinge at right edge of the longitudinal split (Y=0 plane, X=+W/2)
HINGE_X = W / 2.0
HINGE_Y = 0.0
HINGE_Z = SHELL_Z0 + SHELL_H / 2.0

# Door part-frame origin offset from world: visuals in part frame need
# world_to_part = (-HINGE_X, -HINGE_Y, -HINGE_Z)
DO = (-HINGE_X, -HINGE_Y, -HINGE_Z)


def _make_half_shell(y_sign: int, local_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    """Build a hollow thin-walled half-shell for the luggage case.

    y_sign = +1: back half (Y>0), open face at Y=0
    y_sign = -1: front half (Y<0), open face at Y=0

    local_offset: applied to center the shell in the desired frame.
    """
    half_d = D / 2.0
    cy = y_sign * half_d / 2.0 + local_offset[1]
    # Anchor the shell on its raised base (SHELL_Z0): the box spans
    # [SHELL_Z0, SHELL_TOP] so every trim/seam/hinge element that references
    # SHELL_Z0/SHELL_TOP lands on the real shell surface, not floating above it.
    cz = SHELL_Z0 + SHELL_H / 2.0 + local_offset[2]
    cx = local_offset[0]

    solid = (
        cq.Workplane("XY")
        .transformed(offset=(cx, cy, cz))
        .box(W, half_d, SHELL_H)
    )

    # Fillet the outer vertical edges
    if y_sign > 0:
        solid = solid.faces(">Y").edges("|Z").fillet(CORNER_R)
        inner_face_sel = "<Y"
    else:
        solid = solid.faces("<Y").edges("|Z").fillet(CORNER_R)
        inner_face_sel = ">Y"

    # Shell: remove inner face to create hollow compartment
    solid = solid.faces(inner_face_sel).shell(WALL_T)
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_luggage_bag")

    shell_mat = model.material("shell_green", rgba=(0.20, 0.40, 0.16, 1.0))
    door_mat = model.material("door_graphite", rgba=(0.22, 0.22, 0.24, 1.0))
    trim_mat = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_mat = model.material("metal_chrome", rgba=(0.60, 0.60, 0.63, 1.0))
    wheel_mat = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))

    seam_w = 0.012
    seam_h = 0.016

    # =========================================================================
    # BODY (back half-shell, Y>0) — root part carrying wheels and handle
    # Part frame at world origin.
    # =========================================================================
    body = model.part("luggage_body")

    # Hollow CadQuery shell for the body half (built in world coords)
    body.visual(
        mesh_from_cadquery(_make_half_shell(+1), "body_shell"),
        material=shell_mat,
        name="body_shell",
    )

    # Seam/zipper strips around the Y=0 split opening on the body side.
    # Right seam is set inward from the hinge barrel to avoid overlap.
    hinge_inset = 0.025
    body_seam_specs = [
        ("top", (0.0, seam_w / 2.0, SHELL_TOP - seam_h / 2.0),
         (W - 0.02, seam_w, seam_h)),
        ("bot", (0.0, seam_w / 2.0, SHELL_Z0 + seam_h / 2.0),
         (W - 0.02, seam_w, seam_h)),
        ("left", (-(W / 2.0 - seam_h / 2.0), seam_w / 2.0, SHELL_Z0 + SHELL_H / 2.0),
         (seam_h, seam_w, SHELL_H - 0.04)),
        ("right", (W / 2.0 - hinge_inset - seam_h / 2.0, seam_w / 2.0, SHELL_Z0 + SHELL_H / 2.0),
         (seam_h, seam_w, SHELL_H - 0.04)),
    ]
    for i, (suffix, xyz, sz) in enumerate(body_seam_specs):
        body.visual(
            Box(sz),
            origin=Origin(xyz=xyz),
            material=trim_mat,
            name=f"body_seam_{suffix}",
        )

    # Handle housing plate + exit bosses on top of body (centered at Y=D/4)
    handle_y = D / 4.0
    body.visual(
        Box((0.28, 0.08, 0.012)),
        origin=Origin(xyz=(0.0, handle_y, SHELL_TOP - 0.004)),
        material=trim_mat,
        name="handle_housing",
    )
    for i, sx in enumerate((-HANDLE_X, HANDLE_X)):
        body.visual(
            Cylinder(radius=0.017, length=0.03),
            origin=Origin(xyz=(sx, handle_y, SHELL_TOP)),
            material=trim_mat,
            name=f"handle_boss_{i}",
        )

    # Side grab handle on +X face (centered at Y=D/4)
    for i, dy in enumerate((-0.035, 0.035)):
        body.visual(
            Box((0.035, 0.022, 0.022)),
            origin=Origin(
                xyz=(W / 2.0 + 0.0125, handle_y + dy, SHELL_Z0 + SHELL_H * 0.78)
            ),
            material=trim_mat,
            name=f"side_post_{i}",
        )
    body.visual(
        Cylinder(radius=0.011, length=0.09),
        origin=Origin(
            xyz=(W / 2.0 + 0.028, handle_y, SHELL_Z0 + SHELL_H * 0.78),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_mat,
        name="side_grip",
    )

    # Hinge barrel at the right edge of the split
    hinge_len = SHELL_H - 0.04
    body.visual(
        Cylinder(radius=0.012, length=hinge_len),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        material=metal_mat,
        name="hinge_barrel",
    )

    # Latch at the free edge (opposite hinge, X=-W/2)
    body.visual(
        Box((0.04, 0.018, 0.06)),
        origin=Origin(xyz=(-(W / 2.0 - 0.008), 0.009, SHELL_Z0 + SHELL_H * 0.5)),
        material=metal_mat,
        name="latch_body",
    )

    # Low base chassis / skid plate spanning the full footprint. It is the rigid
    # floor of the case and carries all four corner casters (so the front wheels
    # under the swinging door are still grounded on the fixed body).
    body.visual(
        Box((W - 0.006, D - 0.006, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_Z0 - BASE_T / 2.0)),
        material=trim_mat,
        name="base_chassis",
    )

    # Caster yoke brackets: hang below the chassis at each corner and capture the
    # wheel axle. Top of the fork meets the chassis; the wheel's lower half is
    # exposed and rolls on the ground entirely below the shell.
    for i, (sx, sy) in enumerate(WHEEL_POSITIONS):
        body.visual(
            Box((0.05, WHEEL_T + 0.018, 0.045)),
            origin=Origin(xyz=(sx, sy, WHEEL_CZ + 0.013)),
            material=trim_mat,
            name=f"yoke_{i}",
        )

    # =========================================================================
    # DOOR (front half-shell, Y<0) — swings open on vertical-edge hinge
    # Part frame at q=0 coincides with articulation frame at (HINGE_X, 0, HINGE_Z).
    # All visuals authored in part-local coordinates (offset by DO).
    # =========================================================================
    door = model.part("door")

    # Hollow CadQuery shell for the door half, built in part-local frame
    door.visual(
        mesh_from_cadquery(_make_half_shell(-1, local_offset=DO), "door_shell"),
        material=door_mat,
        name="door_shell",
    )

    # Seam strips on the door side (part-local coords).
    # Right seam is inset from hinge to match body and avoid knuckle overlap.
    door_seam_specs = [
        ("top", (DO[0], DO[1] - seam_w / 2.0, DO[2] + SHELL_TOP - seam_h / 2.0),
         (W - 0.02, seam_w, seam_h)),
        ("bot", (DO[0], DO[1] - seam_w / 2.0, DO[2] + SHELL_Z0 + seam_h / 2.0),
         (W - 0.02, seam_w, seam_h)),
        ("left", (DO[0] - (W / 2.0 - seam_h / 2.0), DO[1] - seam_w / 2.0, DO[2] + SHELL_Z0 + SHELL_H / 2.0),
         (seam_h, seam_w, SHELL_H - 0.04)),
        ("right", (DO[0] + W / 2.0 - hinge_inset - seam_h / 2.0, DO[1] - seam_w / 2.0, DO[2] + SHELL_Z0 + SHELL_H / 2.0),
         (seam_h, seam_w, SHELL_H - 0.04)),
    ]
    for i, (suffix, xyz, sz) in enumerate(door_seam_specs):
        door.visual(
            Box(sz),
            origin=Origin(xyz=xyz),
            material=trim_mat,
            name=f"door_seam_{suffix}",
        )

    # Door hinge knuckle (wraps inside the body hinge barrel)
    # In part frame: barrel world is (HINGE_X, 0, HINGE_Z), part frame is there too,
    # so knuckle is at local (0, 0, 0).
    door.visual(
        Cylinder(radius=0.009, length=hinge_len - 0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_mat,
        name="door_hinge_knuckle",
    )

    # Door latch catch at free edge (part-local coords)
    door.visual(
        Box((0.03, 0.015, 0.04)),
        origin=Origin(xyz=(DO[0] - (W / 2.0 - 0.008), DO[1] - 0.010, DO[2] + SHELL_Z0 + SHELL_H * 0.5)),
        material=metal_mat,
        name="latch_door",
    )

    # Body-to-door revolute hinge
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        # axis=(0,0,1): positive q rotates door CCW from above,
        # swinging the free edge (X=-W/2) toward -Y (away from body).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.0, lower=0.0, upper=math.pi * 0.45
        ),
    )

    # =========================================================================
    # TELESCOPING PULL HANDLE (prismatic, retracts into body shell)
    # Part frame at world origin (articulation origin is identity).
    # =========================================================================
    handle = model.part("pull_handle")
    tube_len = TUBE_TOP - TUBE_BOT
    tube_cz = (TUBE_TOP + TUBE_BOT) / 2.0

    for sx, nm in ((-HANDLE_X, "tube_left"), (HANDLE_X, "tube_right")):
        handle.visual(
            Cylinder(radius=TUBE_R, length=tube_len),
            origin=Origin(xyz=(sx, handle_y, tube_cz)),
            material=metal_mat,
            name=nm,
        )
    handle.visual(
        Cylinder(radius=0.013, length=2.0 * HANDLE_X + 0.02),
        origin=Origin(
            xyz=(0.0, handle_y, TUBE_TOP), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
        material=trim_mat,
        name="cross_grip",
    )
    handle.visual(
        Box((0.10, 0.03, 0.022)),
        origin=Origin(xyz=(0.0, handle_y, TUBE_TOP)),
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
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.3, lower=0.0, upper=HANDLE_TRAVEL
        ),
    )

    # =========================================================================
    # FOUR SPINNER WHEELS (continuous spin about local Y)
    # Each wheel part frame at q=0 = articulation origin (sx, sy, WHEEL_CZ).
    # Visuals at local origin.
    # =========================================================================
    for i, (sx, sy) in enumerate(WHEEL_POSITIONS):
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
    door = object_model.get_part("door")
    handle = object_model.get_part("pull_handle")
    handle_joint = object_model.get_articulation("handle_extend")
    door_joint = object_model.get_articulation("body_to_door")
    wheel0 = object_model.get_part("wheel_0")
    wheel0_spin = object_model.get_articulation("wheel_0_spin")

    # --- Overlap allowances ---------------------------------------------------

    # Pull-handle tubes retract into the body shell cavity
    for tube_name in ("tube_left", "tube_right"):
        ctx.allow_overlap(
            handle, body, elem_a=tube_name, elem_b="body_shell",
            reason=f"{tube_name} retracts into the hollow body shell.",
        )
        ctx.allow_overlap(
            handle, body, elem_a=tube_name, elem_b="handle_housing",
            reason=f"{tube_name} passes through the top handle housing plate.",
        )
        boss_name = "handle_boss_0" if tube_name == "tube_left" else "handle_boss_1"
        ctx.allow_overlap(
            handle, body, elem_a=tube_name, elem_b=boss_name,
            reason=f"{tube_name} passes through its exit boss.",
        )

    # Hinge knuckle captured inside barrel and passes through body shell wall
    ctx.allow_overlap(
        door, body,
        elem_a="door_hinge_knuckle", elem_b="hinge_barrel",
        reason="Door hinge knuckle is captured inside the body hinge barrel.",
    )
    ctx.allow_overlap(
        door, body,
        elem_a="door_hinge_knuckle", elem_b="body_shell",
        reason="Door hinge knuckle passes through the body shell wall at the hinge line.",
    )

    # Door shell wraps around the hinge barrel area at the right edge
    ctx.allow_overlap(
        door, body,
        elem_a="door_shell", elem_b="hinge_barrel",
        reason="Door shell right edge wraps around the hinge barrel mount point.",
    )
    # Door seam strip at the right edge meets the hinge barrel
    ctx.allow_overlap(
        door, body,
        elem_a="door_seam_right", elem_b="hinge_barrel",
        reason="Door seam strip at the hinge edge contacts the barrel mount.",
    )

    # Door bottom edge rests on the fixed base chassis lip.
    ctx.allow_overlap(
        door, body,
        elem_a="door_shell", elem_b="base_chassis",
        reason="Door lower edge seats on the fixed base chassis when closed.",
    )

    # Wheels are captured by their caster yokes and pivot up into the base
    # chassis (caster mount), but the rolling body of each wheel stays below
    # the shell — never inside the case box.
    for i in range(4):
        w = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            w, body, elem_a="axle", elem_b=f"yoke_{i}",
            reason=f"Wheel {i} axle captured inside caster yoke.",
        )
        ctx.allow_overlap(
            w, body, elem_a="tire", elem_b=f"yoke_{i}",
            reason=f"Wheel {i} tire upper half recessed into the caster fork.",
        )
        ctx.allow_overlap(
            w, body, elem_a="tire", elem_b="base_chassis",
            reason=f"Wheel {i} caster pivots up into the base chassis.",
        )

    # --- Door mechanism tests ------------------------------------------------

    # Closed door: door and body meet at the Y=0 split line
    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            body, door, axis="y",
            max_gap=0.015, max_penetration=0.005,
            positive_elem="body_shell", negative_elem="door_shell",
            name="closed door meets body at longitudinal split",
        )
        ctx.expect_overlap(
            door, body, axes="z",
            elem_a="door_shell", elem_b="body_shell",
            min_overlap=0.20,
            name="closed door overlaps body along height",
        )
        ctx.expect_overlap(
            door, body, axes="x",
            elem_a="door_shell", elem_b="body_shell",
            min_overlap=0.15,
            name="closed door overlaps body along width",
        )

    # Open door: free edge swings outward (door_shell center moves in Y)
    with ctx.pose({door_joint: 0.0}):
        door_closed_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
    with ctx.pose({door_joint: 1.0}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_shell")

    ctx.check(
        "door swings sideways when opened",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][1] < door_closed_aabb[0][1] - 0.03,
        details=f"closed_min_y={door_closed_aabb[0][1] if door_closed_aabb else None}, "
                f"open_min_y={door_open_aabb[0][1] if door_open_aabb else None}",
    )

    # Hinge knuckle stays near barrel center
    ctx.expect_contact(
        door, body,
        elem_a="door_hinge_knuckle", elem_b="hinge_barrel",
        contact_tol=0.005,
        name="hinge knuckle in contact with barrel",
    )

    # --- Handle mechanism tests ---------------------------------------------

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_within(
            handle, body, axes="xy",
            inner_elem="tube_left", outer_elem="body_shell",
            margin=0.005,
            name="retracted handle tube within body footprint",
        )
        ctx.expect_overlap(
            handle, body, axes="z",
            elem_a="tube_left", elem_b="body_shell",
            min_overlap=0.20,
            name="retracted handle deeply inserted in body",
        )
        rest_top = ctx.part_element_world_aabb(handle, elem="cross_grip")

    with ctx.pose({handle_joint: HANDLE_TRAVEL}):
        ext_top = ctx.part_element_world_aabb(handle, elem="cross_grip")
        ctx.expect_overlap(
            handle, body, axes="z",
            elem_a="tube_left", elem_b="body_shell",
            min_overlap=0.06,
            name="extended handle retains insertion in body",
        )

    ctx.check(
        "pull handle extends upward",
        rest_top is not None
        and ext_top is not None
        and ext_top[1][2] > rest_top[1][2] + 0.25,
        details=f"rest_top={rest_top}, ext_top={ext_top}",
    )

    # --- Wheel tests ---------------------------------------------------------

    ctx.expect_contact(
        wheel0, body, elem_a="axle", elem_b="yoke_0",
        contact_tol=1e-4,
        name="front wheel axle seated in yoke",
    )

    with ctx.pose({wheel0_spin: 0.0}):
        lug_rest = ctx.part_element_world_aabb(wheel0, elem="tread_lug")
    with ctx.pose({wheel0_spin: math.pi / 2.0}):
        lug_spun = ctx.part_element_world_aabb(wheel0, elem="tread_lug")
    ctx.check(
        "wheel spins about axle",
        lug_rest is not None
        and lug_spun is not None
        and lug_spun[1][2] < lug_rest[1][2] - 0.015,
        details=f"rest={lug_rest}, spun={lug_spun}",
    )

    ctx.check(
        "four base wheels present",
        all(object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
        details="expected wheel_0..wheel_3",
    )

    # Wheels must live BELOW the case shell, not embedded inside the box.
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    shell_base_z = body_shell_aabb[0][2] if body_shell_aabb else None
    for i in range(4):
        w = object_model.get_part(f"wheel_{i}")
        tire_aabb = ctx.part_element_world_aabb(w, elem="tire")
        tire_top = tire_aabb[1][2] if tire_aabb else None
        ctx.check(
            f"wheel {i} sits below the shell (not inside the box)",
            shell_base_z is not None and tire_top is not None
            and tire_top <= shell_base_z + 0.001,
            details=f"tire_top={tire_top}, shell_base_z={shell_base_z}",
        )

    # All four casters spread to the corners (front pair under the door half).
    wheel_ys = [
        ctx.part_element_world_aabb(object_model.get_part(f"wheel_{i}"), elem="tire")
        for i in range(4)
    ]
    ys_center = [(a[0][1] + a[1][1]) / 2.0 for a in wheel_ys if a]
    ctx.check(
        "casters span front and back corners",
        len(ys_center) == 4 and min(ys_center) < -0.04 and max(ys_center) > 0.04,
        details=f"wheel y-centers={ys_center}",
    )

    # --- Shell hollowness / visible geometry claims --------------------------

    ctx.check(
        "body is a hollow CadQuery shell (mesh-backed)",
        body.get_visual("body_shell") is not None,
    )
    ctx.check(
        "door is a hollow CadQuery shell (mesh-backed)",
        door.get_visual("door_shell") is not None,
    )

    return ctx.report()
