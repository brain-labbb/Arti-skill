from __future__ import annotations

# White 20-foot intermodal shipping container.
# Frame: container long axis along +X (cargo-door end at +X), width along Y,
# height along Z. Container interior floor at z=0; body spans z in [0, H].
# Corrugated steel walls (vertical corrugations on the two long side walls and
# on the front end wall), 8 corner castings, a steel floor, and at the +X end
# two corrugated cargo doors. The left door is hinged on its +Y edge and the
# right door on its -Y edge; each door carries two vertical locking rods, and
# each rod has a cam-handle lever (children of the door) plus keeper brackets.
#
# Articulations:
#   - door_l: REVOLUTE about the +Y vertical hinge edge, swings open 0..120 deg
#   - door_r: REVOLUTE about the -Y vertical hinge edge, swings open 0..120 deg
#   - handle_l0/l1/r0/r1: REVOLUTE about each rod's vertical axis, 0..110 deg

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
# Closed doors sit just inside the doorway, behind the +X corner castings so
# their frames clear the castings. DOOR_X is the x-center of the door panels.
DOOR_X = L / 2.0 - CORNER - DOOR_T / 2.0
BODY_X1 = L / 2.0 - DOOR_T  # +X face of the body opening
BODY_X0 = -L / 2.0

CORR_PITCH = 0.30  # corrugation pitch
CORR_DEPTH = 0.04  # corrugation rib depth (proud of base skin)
CORR_WIDTH = 0.12  # corrugation rib width


def _corrugated_wall(length_x: float, height_z: float, base_y: float, depth_sign: float) -> MeshGeometry:
    """Vertical-corrugation side wall lying in the XZ plane at y=base_y.

    A thin base skin plus vertical ribs (boxes spanning z) repeated along X.
    depth_sign = +1 ribs protrude toward +Y, -1 toward -Y.
    """
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
    corrugation ribs facing +X (outward). Door local frame: thickness along X.
    """
    # base skin
    geo = BoxGeometry((DOOR_T * 0.45, width_y, height_z))
    # peripheral frame (rectangular tube) raised proud of the skin on +X side
    fr_t = 0.07
    fr_d = DOOR_T  # frame depth (full door thickness)
    # left/right vertical frame members (along Z)
    for sy in (-1.0, 1.0):
        m = BoxGeometry((fr_d, fr_t, height_z))
        m.translate(0.0, sy * (width_y / 2.0 - fr_t / 2.0), 0.0)
        geo.merge(m)
    # top/bottom horizontal frame members (along Y)
    for sz in (-1.0, 1.0):
        m = BoxGeometry((fr_d, width_y - 2 * fr_t, fr_t))
        m.translate(0.0, 0.0, sz * (height_z / 2.0 - fr_t / 2.0))
        geo.merge(m)
    # vertical corrugation ribs across the inner field, facing +X
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

    body_len = BODY_X1 - BODY_X0  # length of the closed body section
    body_cx = (BODY_X0 + BODY_X1) / 2.0  # x-center of the body section

    # --- floor (steel plate) ---
    floor = BoxGeometry((body_len, W, 0.10))
    floor.translate(body_cx, 0.0, 0.05)
    body.visual(mesh_from_geometry(floor, "floor"), material=grey, name="floor")

    # --- roof (slightly domed flat panel) ---
    roof = BoxGeometry((body_len, W, 0.05))
    roof.translate(body_cx, 0.0, H - 0.025)
    body.visual(mesh_from_geometry(roof, "roof"), material=white, name="roof")

    # --- two long corrugated side walls (at +/- Y) ---
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

    # --- door header & sill rails at the +X opening (frame the doorway) ---
    header = BoxGeometry((0.10, W, 0.14))
    header.translate(BODY_X1 - 0.05, 0.0, H - 0.07)
    body.visual(mesh_from_geometry(header, "door_header"), material=grey, name="door_header")
    sill = BoxGeometry((0.10, W, 0.14))
    sill.translate(BODY_X1 - 0.05, 0.0, 0.07)
    body.visual(mesh_from_geometry(sill, "door_sill"), material=grey, name="door_sill")
    # vertical corner posts at the doorway (carry the hinges)
    for sy, nm in ((1.0, "post_p"), (-1.0, "post_n")):
        post = BoxGeometry((0.12, 0.14, H))
        post.translate(BODY_X1 - 0.05, sy * (W / 2.0 - 0.07), H / 2.0)
        body.visual(mesh_from_geometry(post, nm), material=grey, name=nm)

    # --- 8 corner castings (dark steel cubes at the 8 box corners) ---
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
    # DOORS: two corrugated panels at the +X end.
    # Each door is a child of body via a REVOLUTE joint on its outboard
    # vertical edge. Door local frame: panel thickness along X, panel plane YZ.
    # Local origin at the door's HINGE edge so rotation swings the free edge out.
    # =====================================================================
    # Each door covers a bit less than half the opening, leaving a real seam
    # gap at the center and clearance below the header / above the sill.
    door_half_w = (W - 0.16) / 2.0
    door_h = H - 0.40

    door_specs = [
        # (name, hinge_sign): hinge_sign=+1 hinged on +Y edge (left), -1 on -Y edge (right)
        ("door_l", 1.0),
        ("door_r", -1.0),
    ]

    handle_count = 0
    for dname, hinge_sign in door_specs:
        door = model.part(dname)
        # Panel geometry is authored so that its HINGE edge is at local y=0 and
        # the free edge is at local y = -hinge_sign*door_half_w (toward center).
        # Build the panel centered, then shift it so y=0 is the hinge edge.
        panel = _corrugated_door_panel(door_half_w, door_h)
        # shift so hinge edge (the outboard edge) lands at local y=0:
        # outboard edge of a centered panel is at y = +hinge_sign*door_half_w/2... build centered then translate
        panel.translate(0.0, -hinge_sign * (door_half_w / 2.0), 0.0)
        door.visual(mesh_from_geometry(panel, "door_panel"), material=white, name="door_panel")

        door.inertial = Inertial.from_geometry(
            Box((DOOR_T, door_half_w, door_h)),
            mass=80.0,
            origin=Origin(xyz=(0.0, -hinge_sign * door_half_w / 2.0, 0.0)),
        )

        # Hinge at the outboard vertical edge of the opening. The door spans
        # from a small center seam out to this hinge line.
        hinge_y = hinge_sign * (door_half_w + 0.04)
        # left door (hinge_sign=+1) should open by rotating so free edge moves
        # in +X (outward). The door plane is YZ at x=DOOR_X. With axis +Z and
        # right-hand rule, positive q moves +Y... we want the free edge (toward
        # -Y for left door) to swing toward +X. Choose axis sign per door.
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

        # ---- locking rods + cam handles on this door ----
        # Two vertical rods per door, located near the free edge and near mid.
        # Rod local Y positions (in door frame, hinge edge at y=0).
        free_dir = -hinge_sign  # toward container center
        rod_ys = [
            free_dir * (door_half_w * 0.30),
            free_dir * (door_half_w * 0.78),
        ]
        # Rods run vertically just proud of the outer (+X) door face; they
        # touch the corrugation ribs / skin so they are physically supported.
        rod_x = DOOR_T * 0.225 + CORR_DEPTH - 0.012
        for ridx, ry in enumerate(rod_ys):
            # locking rod: long vertical cylinder along Z
            rod = CylinderGeometry(0.018, door_h - 0.10, radial_segments=16)
            rod.translate(rod_x, ry, 0.0)
            door.visual(mesh_from_geometry(rod, f"rod_{ridx}"), material=dark, name=f"rod_{ridx}")
            # keeper brackets (top & bottom) that capture the rod against the door
            for sz in (1.0, -1.0):
                kp = BoxGeometry((0.05, 0.06, 0.05))
                kp.translate(rod_x - 0.02, ry, sz * (door_h / 2.0 - 0.18))
                door.visual(
                    mesh_from_geometry(kp, f"keeper_{ridx}_{0 if sz > 0 else 1}"),
                    material=grey,
                    name=f"keeper_{ridx}_{0 if sz > 0 else 1}",
                )

            # ---- cam handle: child of the door, rotates about the rod axis ----
            hsuffix = ("l" if hinge_sign > 0 else "r") + str(ridx)
            hname = f"handle_{hsuffix}"
            handle = model.part(hname)
            # Cam hub clamps the rod and stands off the door face (+X), so the
            # lever bar can sweep clear of the corrugated panel ribs.
            hub = CylinderGeometry(0.022, 0.06, radial_segments=16).rotate_y(math.pi / 2.0)
            hub.translate(0.04, 0.0, 0.0)
            handle.visual(mesh_from_geometry(hub, "hub"), material=dark, name="hub")
            # off-axis lever bar pointing outward (+X), held clear of the door
            # face so rotation about the rod axis (Z) is clearly detectable.
            lever = BoxGeometry((0.14, 0.045, 0.035))
            lever.translate(0.13, 0.0, 0.0)
            handle.visual(mesh_from_geometry(lever, "lever"), material=dark, name="lever")
            # grip knob at the end of the lever
            grip = CylinderGeometry(0.025, 0.10, radial_segments=12).rotate_y(math.pi / 2.0)
            grip.translate(0.20, 0.0, 0.0)
            handle.visual(mesh_from_geometry(grip, "grip"), material=grey, name="grip")

            handle.inertial = Inertial.from_geometry(
                Box((0.22, 0.06, 0.06)), mass=1.2, origin=Origin(xyz=(0.11, 0.0, 0.0))
            )
            # Handle mounted on the rod at mid-height, rotating about the rod (Z).
            # The hub wraps the rod (intentional overlap); the lever sticks out.
            model.articulation(
                f"{dname}_to_{hname}",
                ArticulationType.REVOLUTE,
                parent=door,
                child=handle,
                origin=Origin(xyz=(rod_x, ry, 0.05)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=20.0, velocity=3.0, lower=0.0, upper=math.radians(110.0)
                ),
            )
            handle_count += 1

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_l = object_model.get_part("door_l")
    door_r = object_model.get_part("door_r")
    hinge_l = object_model.get_articulation("body_to_door_l")
    hinge_r = object_model.get_articulation("body_to_door_r")

    # ---- container is a long box: length(X) >> width(Y), and tall ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "container is a long box (X longest, taller than wide is false; X>Y, X>Z)",
        body_ext[0] > body_ext[1] + 1.0 and body_ext[0] > body_ext[2] + 1.0,
        details=f"body extents={body_ext}",
    )

    # ---- doors are at the +X (cargo) end of the container ----
    for d, nm in ((door_l, "door_l"), (door_r, "door_r")):
        pos = ctx.part_world_position(d)
        ctx.check(
            f"{nm} is at the +X cargo end",
            pos is not None and pos[0] > L / 2.0 - 0.3,
            details=f"{nm} origin={pos}",
        )

    # ---- doors hinge on the doorway corner posts (intentional mount overlap) ----
    ctx.allow_overlap(
        door_l, body,
        elem_a="door_panel", elem_b="post_p",
        reason="Left door's hinge edge frame seats against the +Y doorway corner post that carries its hinge.",
    )
    ctx.allow_overlap(
        door_r, body,
        elem_a="door_panel", elem_b="post_n",
        reason="Right door's hinge edge frame seats against the -Y doorway corner post that carries its hinge.",
    )
    ctx.expect_contact(door_l, body, name="left door seated against doorway")
    ctx.expect_contact(door_r, body, name="right door seated against doorway")

    # ---- each cam handle is seated on its rod against the door face ----
    for hsuffix, ridx in (("l0", 0), ("l1", 1), ("r0", 0), ("r1", 1)):
        hname = f"handle_{hsuffix}"
        dname = "door_l" if hsuffix.startswith("l") else "door_r"
        d = object_model.get_part(dname)
        h = object_model.get_part(hname)
        ctx.allow_overlap(
            d, h, elem_a=f"rod_{ridx}", elem_b="hub",
            reason="The cam-handle hub is intentionally seated around the locking rod it actuates.",
        )
        ctx.allow_overlap(
            d, h, elem_a=f"rod_{ridx}", elem_b="lever",
            reason="The cam lever passes over its locking rod where it clamps onto the rod.",
        )
        ctx.allow_overlap(
            d, h, elem_a="door_panel", elem_b="hub",
            reason="The handle hub sits flush against the corrugated door face beside the rod.",
        )

    # ---- both doors swing open: free edge moves out in +X when opened ----
    for d, hinge, nm in ((door_l, hinge_l, "door_l"), (door_r, hinge_r, "door_r")):
        x0 = _ext(ctx.part_world_aabb(d))[0]
        with ctx.pose({hinge: math.radians(110.0)}):
            x1 = _ext(ctx.part_world_aabb(d))[0]
        ctx.check(
            f"{nm} swings open about its side hinge (free edge moves outward in +X)",
            x1 > x0 + 0.4,
            details=f"{nm} X-extent rest={x0:.3f} open={x1:.3f}",
        )

    # ---- four cam handles each rotate about their rod (lever sweeps) ----
    for hsuffix in ("l0", "l1", "r0", "r1"):
        hname = f"handle_{hsuffix}"
        dname = "door_l" if hsuffix.startswith("l") else "door_r"
        handle = object_model.get_part(hname)
        joint = object_model.get_articulation(f"{dname}_to_{hname}")
        # lever rides with the door and is mounted on the rod (contact w/ door)
        ctx.expect_contact(handle, object_model.get_part(dname), name=f"{hname} mounted on its door rod")
        ext0 = _ext(ctx.part_world_aabb(handle))
        with ctx.pose({joint: math.radians(90.0)}):
            ext90 = _ext(ctx.part_world_aabb(handle))
        # rotating the lever about Z swaps its X/Y footprint detectably
        ctx.check(
            f"{hname} rotates about its rod axis (lever sweeps)",
            abs(ext90[1] - ext0[1]) > 0.05 or abs(ext90[0] - ext0[0]) > 0.05,
            details=f"{hname} rest_ext={ext0} turned_ext={ext90}",
        )

    return ctx.report()


object_model = build_object_model()
