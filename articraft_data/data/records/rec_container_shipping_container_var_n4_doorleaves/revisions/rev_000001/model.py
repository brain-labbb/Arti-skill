from __future__ import annotations

# White 20-foot intermodal shipping container with four narrow corrugated
# cargo-door leaves at the +X end. Each leaf carries its own locking rod and
# cam handle. Leaves alternate hinge sides: even-index leaves hinge on their
# +Y edge, odd-index leaves on their -Y edge.
#
# Frame: container long axis along +X (cargo-door end at +X), width along Y,
# height along Z. Interior floor at z=0; body spans z in [0, H].
#
# Articulations (all REVOLUTE):
#   - body_to_door_0..3: swing open 0..120 deg about each leaf's hinge edge
#   - door_i_to_handle_i: cam handle rotation about its rod axis, 0..110 deg

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- container outer dimensions (meters, realistic ISO 20ft) ----
L = 6.06  # length along X
W = 2.44  # width along Y
H = 2.59  # height along Z

WALL_T = 0.035  # nominal wall/skin thickness
CORNER = 0.16  # corner casting cube size

# Body occupies x in [BODY_X0, BODY_X1]; the doors occupy the rest at +X.
DOOR_T = 0.06  # door panel thickness
DOOR_X = L / 2.0 - CORNER - DOOR_T / 2.0
BODY_X1 = L / 2.0 - DOOR_T
BODY_X0 = -L / 2.0

CORR_PITCH = 0.30
CORR_DEPTH = 0.04
CORR_WIDTH = 0.12

DOOR_COUNT = 4


def _corrugated_wall(length_x: float, height_z: float, base_y: float, depth_sign: float) -> MeshGeometry:
    """Vertical-corrugation side wall lying in the XZ plane at y=base_y."""
    geo = BoxGeometry((length_x, WALL_T, height_z))
    geo.translate(0.0, base_y, 0.0)
    n = max(1, int(length_x / CORR_PITCH))
    x0 = -length_x / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        x = x0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_WIDTH, CORR_DEPTH, height_z * 0.96))
        rib.translate(x, base_y + depth_sign * (WALL_T / 2.0 + CORR_DEPTH / 2.0), 0.0)
        geo.merge(rib)
    return geo


def _corrugated_end_wall(width_y: float, height_z: float, base_x: float, depth_sign: float) -> MeshGeometry:
    """Front (-X) end wall in the YZ plane at x=base_x with vertical ribs along Y."""
    geo = BoxGeometry((WALL_T, width_y, height_z))
    geo.translate(base_x, 0.0, 0.0)
    n = max(1, int(width_y / CORR_PITCH))
    y0 = -width_y / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        y = y0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_DEPTH, CORR_WIDTH, height_z * 0.96))
        rib.translate(base_x + depth_sign * (WALL_T / 2.0 + CORR_DEPTH / 2.0), y, 0.0)
        geo.merge(rib)
    return geo


def _corrugated_door_panel(width_y: float, height_z: float) -> MeshGeometry:
    """A single corrugated cargo door panel centered at its local origin.

    Panel plane is YZ (thin along X). Includes a peripheral frame and vertical
    corrugation ribs facing +X (outward).
    """
    geo = BoxGeometry((DOOR_T * 0.45, width_y, height_z))
    fr_t = 0.07
    fr_d = DOOR_T
    for sy in (-1.0, 1.0):
        m = BoxGeometry((fr_d, fr_t, height_z))
        m.translate(0.0, sy * (width_y / 2.0 - fr_t / 2.0), 0.0)
        geo.merge(m)
    for sz in (-1.0, 1.0):
        m = BoxGeometry((fr_d, width_y - 2 * fr_t, fr_t))
        m.translate(0.0, 0.0, sz * (height_z / 2.0 - fr_t / 2.0))
        geo.merge(m)
    inner_w = width_y - 2 * fr_t
    n = max(1, int(inner_w / CORR_PITCH))
    y0 = -inner_w / 2.0 + CORR_PITCH / 2.0
    for i in range(n):
        y = y0 + i * CORR_PITCH
        rib = BoxGeometry((CORR_DEPTH, CORR_WIDTH, height_z - 2 * fr_t - 0.02))
        rib.translate(DOOR_T * 0.225 + CORR_DEPTH / 2.0, y, 0.0)
        geo.merge(rib)
    return geo


def _corner_casting() -> MeshGeometry:
    return BoxGeometry((CORNER, CORNER, CORNER))


def _build_handle(model, door, dname: str, hname: str, rod_x: float, rod_y: float, *, dark, grey):
    """Create a cam-handle part articulated to its parent door about the rod axis."""
    handle = model.part(hname)

    hub = CylinderGeometry(0.022, 0.06, radial_segments=16).rotate_y(math.pi / 2.0)
    hub.translate(0.04, 0.0, 0.0)
    handle.visual(mesh_from_geometry(hub, "hub"), material=dark, name="hub")

    lever = BoxGeometry((0.14, 0.045, 0.035))
    lever.translate(0.13, 0.0, 0.0)
    handle.visual(mesh_from_geometry(lever, "lever"), material=dark, name="lever")

    grip = CylinderGeometry(0.025, 0.10, radial_segments=12).rotate_y(math.pi / 2.0)
    grip.translate(0.20, 0.0, 0.0)
    handle.visual(mesh_from_geometry(grip, "grip"), material=grey, name="grip")

    handle.inertial = Inertial.from_geometry(
        Box((0.22, 0.06, 0.06)), mass=1.2, origin=Origin(xyz=(0.11, 0.0, 0.0))
    )

    model.articulation(
        f"{dname}_to_{hname}",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(rod_x, rod_y, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=3.0, lower=0.0, upper=math.radians(110.0)
        ),
    )
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shipping_container")

    white = model.material("white_steel", rgba=(0.90, 0.91, 0.92, 1.0))
    grey = model.material("grey_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("dark_hardware", rgba=(0.18, 0.19, 0.21, 1.0))

    # =====================================================================
    # BODY (root): corrugated box open at the +X end, with floor, roof,
    # front end wall, two side walls, and 8 corner castings.
    # =====================================================================
    body = model.part("body")

    body_len = BODY_X1 - BODY_X0
    body_cx = (BODY_X0 + BODY_X1) / 2.0

    # --- floor ---
    floor = BoxGeometry((body_len, W, 0.10))
    floor.translate(body_cx, 0.0, 0.05)
    body.visual(mesh_from_geometry(floor, "floor"), material=grey, name="floor")

    # --- roof ---
    roof = BoxGeometry((body_len, W, 0.05))
    roof.translate(body_cx, 0.0, H - 0.025)
    body.visual(mesh_from_geometry(roof, "roof"), material=white, name="roof")

    # --- two long corrugated side walls ---
    wall_z_center = H / 2.0
    side_h = H - 0.10
    for sy, nm in ((1.0, "side_wall_p"), (-1.0, "side_wall_n")):
        wall = _corrugated_wall(body_len, side_h, sy * (W / 2.0 - WALL_T / 2.0), sy)
        wall.translate(body_cx, 0.0, wall_z_center)
        body.visual(mesh_from_geometry(wall, nm), material=white, name=nm)

    # --- front end wall (at -X) ---
    end = _corrugated_end_wall(W - 0.10, side_h, BODY_X0 + WALL_T / 2.0, -1.0)
    end.translate(0.0, 0.0, wall_z_center)
    body.visual(mesh_from_geometry(end, "front_wall"), material=white, name="front_wall")

    # --- door header & sill rails at the +X opening ---
    header = BoxGeometry((0.10, W, 0.14))
    header.translate(BODY_X1 - 0.05, 0.0, H - 0.07)
    body.visual(mesh_from_geometry(header, "door_header"), material=grey, name="door_header")
    sill = BoxGeometry((0.10, W, 0.14))
    sill.translate(BODY_X1 - 0.05, 0.0, 0.07)
    body.visual(mesh_from_geometry(sill, "door_sill"), material=grey, name="door_sill")

    # vertical corner posts at the doorway (outer hinge points)
    for sy, nm in ((1.0, "post_p"), (-1.0, "post_n")):
        post = BoxGeometry((0.12, 0.14, H))
        post.translate(BODY_X1 - 0.05, sy * (W / 2.0 - 0.07), H / 2.0)
        body.visual(mesh_from_geometry(post, nm), material=grey, name=nm)

    # center mullion post at y=0 (inner hinge point for doors 1 and 2)
    mullion = BoxGeometry((0.12, 0.14, H))
    mullion.translate(BODY_X1 - 0.05, 0.0, H / 2.0)
    body.visual(mesh_from_geometry(mullion, "mullion"), material=grey, name="mullion")

    # --- 8 corner castings ---
    ci = 0
    for sx in (BODY_X0 + CORNER / 2.0, L / 2.0 - CORNER / 2.0):
        for sy in (-1.0, 1.0):
            for sz in (CORNER / 2.0, H - CORNER / 2.0):
                cc = _corner_casting()
                cc.translate(sx, sy * (W / 2.0 - CORNER / 2.0), sz)
                body.visual(mesh_from_geometry(cc, f"corner_{ci}"), material=dark, name=f"corner_{ci}")
                ci += 1

    body.inertial = Inertial.from_geometry(
        Box((L, W, H)), mass=2200.0, origin=Origin(xyz=(0.0, 0.0, H / 2.0))
    )

    # =====================================================================
    # DOORS: four narrow corrugated leaves at the +X end.
    # Leaves are ordered from +Y to -Y across the opening.
    # Even-index leaves hinge on their +Y edge; odd-index on their -Y edge.
    # This produces two hinge lines: the outer posts and the center mullion.
    # =====================================================================
    opening_w = W - 0.16  # usable doorway width between posts
    leaf_w = opening_w / DOOR_COUNT
    door_h = H - 0.40
    y_open_max = opening_w / 2.0  # +Y edge of the opening

    rod_x = DOOR_T * 0.225 + CORR_DEPTH - 0.012

    for i in range(DOOR_COUNT):
        dname = f"door_{i}"
        hinge_sign = 1.0 if i % 2 == 0 else -1.0

        # Slot boundaries: door 0 at +Y side, door 3 at -Y side
        slot_y_max = y_open_max - i * leaf_w
        slot_y_min = slot_y_max - leaf_w

        # Hinge at the appropriate edge of the slot
        hinge_y = slot_y_max if hinge_sign > 0 else slot_y_min

        door = model.part(dname)

        # Panel geometry: build centered, then shift so hinge edge at local y=0
        panel = _corrugated_door_panel(leaf_w, door_h)
        panel.translate(0.0, -hinge_sign * (leaf_w / 2.0), 0.0)
        door.visual(mesh_from_geometry(panel, "door_panel"), material=white, name="door_panel")

        door.inertial = Inertial.from_geometry(
            Box((DOOR_T, leaf_w, door_h)),
            mass=40.0,
            origin=Origin(xyz=(0.0, -hinge_sign * leaf_w / 2.0, 0.0)),
        )

        # Hinge joint: positive q swings the free edge outward in +X
        axis_z = (0.0, 0.0, hinge_sign)
        model.articulation(
            f"body_to_{dname}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(DOOR_X, hinge_y, H / 2.0)),
            axis=axis_z,
            motion_limits=MotionLimits(
                effort=200.0, velocity=2.0, lower=0.0, upper=math.radians(120.0)
            ),
        )

        # ---- one locking rod + cam handle per leaf ----
        free_dir = -hinge_sign
        rod_y = free_dir * (leaf_w * 0.50)

        rod = CylinderGeometry(0.018, door_h - 0.10, radial_segments=16)
        rod.translate(rod_x, rod_y, 0.0)
        door.visual(mesh_from_geometry(rod, "rod_0"), material=dark, name="rod_0")

        # keeper brackets (top & bottom)
        for sz in (1.0, -1.0):
            kp = BoxGeometry((0.05, 0.06, 0.05))
            kp.translate(rod_x - 0.02, rod_y, sz * (door_h / 2.0 - 0.18))
            keeper_name = f"keeper_0_{0 if sz > 0 else 1}"
            door.visual(
                mesh_from_geometry(kp, keeper_name),
                material=grey,
                name=keeper_name,
            )

        # cam handle as child part
        _build_handle(model, door, dname, f"handle_{i}", rod_x, rod_y, dark=dark, grey=grey)

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    opening_w = W - 0.16
    leaf_w = opening_w / DOOR_COUNT

    # ---- container is a long box: length(X) >> width(Y), X > Z ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "container is a long box (X longest)",
        body_ext[0] > body_ext[1] + 1.0 and body_ext[0] > body_ext[2] + 1.0,
        details=f"body extents={body_ext}",
    )

    # ---- collect all door and handle parts ----
    doors = []
    hinges = []
    handles = []
    handle_joints = []
    for i in range(DOOR_COUNT):
        dname = f"door_{i}"
        hname = f"handle_{i}"
        d = object_model.get_part(dname)
        h = object_model.get_part(hname)
        hinge = object_model.get_articulation(f"body_to_{dname}")
        hjoint = object_model.get_articulation(f"{dname}_to_{hname}")
        doors.append(d)
        hinges.append(hinge)
        handles.append(h)
        handle_joints.append(hjoint)

    # ---- exactly 4 door leaves exist ----
    ctx.check(
        "four door leaves exist",
        len(doors) == 4,
        details=f"found {len(doors)} doors",
    )

    # ---- each door is at the +X cargo end ----
    for i, d in enumerate(doors):
        pos = ctx.part_world_position(d)
        ctx.check(
            f"door_{i} is at the +X cargo end",
            pos is not None and pos[0] > L / 2.0 - 0.3,
            details=f"door_{i} origin={pos}",
        )

    # ---- doors are evenly spaced across the opening in Y ----
    # Use AABB centers (panel centers) rather than part origins (hinge edges)
    panel_centers_y = []
    for d in doors:
        aabb = ctx.part_world_aabb(d)
        panel_centers_y.append((aabb[0][1] + aabb[1][1]) / 2.0)
    for i in range(DOOR_COUNT - 1):
        ctx.check(
            f"door_{i} and door_{i+1} are adjacent in Y",
            abs(abs(panel_centers_y[i] - panel_centers_y[i + 1]) - leaf_w) < 0.05,
            details=f"door_{i} center_y={panel_centers_y[i]:.3f}, door_{i+1} center_y={panel_centers_y[i+1]:.3f}, expected spacing={leaf_w:.3f}",
        )

    # ---- alternating hinge sides: even doors hinge +Y, odd doors hinge -Y ----
    y_open_max = opening_w / 2.0
    for i in range(DOOR_COUNT):
        hinge_sign = 1.0 if i % 2 == 0 else -1.0
        slot_y_max = y_open_max - i * leaf_w
        slot_y_min = slot_y_max - leaf_w
        expected_hinge_y = slot_y_max if hinge_sign > 0 else slot_y_min
        hinge = hinges[i]
        actual_y = hinge.origin.xyz[1]
        ctx.check(
            f"door_{i} hinge on {'+Y' if hinge_sign > 0 else '-Y'} edge at y={expected_hinge_y:.3f}",
            abs(actual_y - expected_hinge_y) < 0.01,
            details=f"expected hinge_y={expected_hinge_y:.3f}, actual={actual_y:.3f}",
        )

    # ---- door-to-body hinge overlap allowances ----
    # door_0 hinges on +Y post, door_3 hinges on -Y post
    # door_1 and door_2 hinge at center mullion
    ctx.allow_overlap(
        doors[0], body,
        elem_a="door_panel", elem_b="post_p",
        reason="door_0 hinge edge seats against the +Y doorway corner post.",
    )
    ctx.allow_overlap(
        doors[3], body,
        elem_a="door_panel", elem_b="post_n",
        reason="door_3 hinge edge seats against the -Y doorway corner post.",
    )
    # inner doors hinge at the center mullion
    for i in (1, 2):
        ctx.allow_overlap(
            doors[i], body,
            elem_a="door_panel", elem_b="mullion",
            reason=f"door_{i} hinge edge seats against the center mullion post.",
        )

    # ---- all doors seated against body frame ----
    for i, d in enumerate(doors):
        ctx.expect_contact(d, body, name=f"door_{i} seated against doorway frame")

    # ---- each door swings open: free edge moves outward in +X ----
    for i, (d, hinge) in enumerate(zip(doors, hinges)):
        x0 = _ext(ctx.part_world_aabb(d))[0]
        with ctx.pose({hinge: math.radians(110.0)}):
            x1 = _ext(ctx.part_world_aabb(d))[0]
        ctx.check(
            f"door_{i} swings open (free edge moves outward in +X)",
            x1 > x0 + 0.2,
            details=f"door_{i} X-extent rest={x0:.3f} open={x1:.3f}",
        )

    # ---- each cam handle mounted on its door and rotates ----
    for i in range(DOOR_COUNT):
        dname = f"door_{i}"
        hname = f"handle_{i}"
        handle = handles[i]
        door = doors[i]
        hjoint = handle_joints[i]

        # handle hub overlaps its rod (intentional seated hardware)
        ctx.allow_overlap(
            door, handle, elem_a="rod_0", elem_b="hub",
            reason=f"handle_{i} hub is seated around its locking rod.",
        )
        ctx.allow_overlap(
            door, handle, elem_a="rod_0", elem_b="lever",
            reason=f"handle_{i} lever passes over its locking rod at the clamp point.",
        )
        ctx.allow_overlap(
            door, handle, elem_a="door_panel", elem_b="hub",
            reason=f"handle_{i} hub sits flush against the corrugated door face.",
        )

        # handle is in contact with its parent door
        ctx.expect_contact(handle, door, name=f"handle_{i} mounted on door_{i}")

        # handle rotation moves the lever visibly
        ext0 = _ext(ctx.part_world_aabb(handle))
        with ctx.pose({hjoint: math.radians(90.0)}):
            ext90 = _ext(ctx.part_world_aabb(handle))
        ctx.check(
            f"handle_{i} rotates about its rod axis (lever sweeps)",
            abs(ext90[1] - ext0[1]) > 0.05 or abs(ext90[0] - ext0[0]) > 0.05,
            details=f"handle_{i} rest_ext={ext0} turned_ext={ext90}",
        )

    # ---- each door has its own independent joint (not mimic) ----
    joint_names = [f"body_to_door_{i}" for i in range(DOOR_COUNT)]
    for jn in joint_names:
        art = object_model.get_articulation(jn)
        ctx.check(
            f"{jn} exists as an independent joint",
            art is not None,
            details=f"{jn} articulation={art}",
        )

    return ctx.report()


object_model = build_object_model()
