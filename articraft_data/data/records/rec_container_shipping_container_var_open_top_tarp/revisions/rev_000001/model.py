from __future__ import annotations

# Open-top 20-foot intermodal shipping container with removable arched tarpaulin
# bows and soft tarp cover. Forked from the flat-roof parent: the fixed steel
# roof panel is removed and replaced by a set of curved steel tube bows
# spanning the container width, repeated along the length, carrying a soft tarp
# cover. One bow is hinged at the +Y side so the top can be opened.
#
# Frame: container long axis along +X (cargo-door end at +X), width along Y,
# height along Z. Interior floor at z=0; body spans z in [0, H].
# Corrugated steel walls, 8 corner castings, steel floor, two corrugated cargo
# doors with locking rods and cam handles -- all identical to the parent.
#
# Articulations:
#   - door_l / door_r: REVOLUTE about side hinge edges, 0..120 deg
#   - handle_l0/l1/r0/r1: REVOLUTE about rod axis, 0..110 deg
#   - body_to_bow_hinged: REVOLUTE about +X at +Y top-rail, 0..80 deg

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
    tube_from_spline_points,
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
BODY_X1 = L / 2.0 - DOOR_T  # +X face of the body opening
BODY_X0 = -L / 2.0

CORR_PITCH = 0.30  # corrugation pitch
CORR_DEPTH = 0.04  # corrugation rib depth (proud of base skin)
CORR_WIDTH = 0.12  # corrugation rib width

# ---- bow and tarp dimensions ----
N_BOWS = 5  # total bows (4 fixed + 1 hinged)
BOW_INSET = 0.30  # inset from body ends to first/last bow
BOW_RISE = 0.18  # arch peak height above base
BOW_TUBE_R = 0.025  # bow tube radius (25mm steel tube)
BOW_SPAN = W - 0.20  # inner span between top rails
TOP_RAIL_W = 0.08  # top rail width (along Y)
TOP_RAIL_H = 0.06  # top rail height (along Z)
BOW_BASE_Z = H  # bows mount at the top of the structure
TARP_THICKNESS = 0.008  # tarp material thickness

# Body section metrics
body_len = BODY_X1 - BODY_X0
body_cx = (BODY_X0 + BODY_X1) / 2.0

# Bow X positions (evenly spaced along body)
_bow_available = body_len - 2.0 * BOW_INSET
_bow_spacing = _bow_available / (N_BOWS - 1)
BOW_POSITIONS_X = [BODY_X0 + BOW_INSET + i * _bow_spacing for i in range(N_BOWS)]


# =====================================================================
# Geometry helpers (shared with parent for walls, doors, castings)
# =====================================================================

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
    """A single corrugated cargo door panel centered at its local origin."""
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


# =====================================================================
# Bow and tarp geometry helpers (new for open-top variant)
# =====================================================================

def _bow_arch_points(half_span: float, rise: float, n: int = 10) -> list[tuple[float, float, float]]:
    """Generate arch path points for a roof bow centered at origin (y from -half_span to +half_span)."""
    points = []
    for i in range(n + 1):
        t = i / n
        angle = math.pi * t
        y = -half_span + 2.0 * half_span * t
        z = rise * math.sin(angle)
        points.append((0.0, y, z))
    return points


def _bow_arch_points_from_hinge(span: float, rise: float, n: int = 10) -> list[tuple[float, float, float]]:
    """Generate arch path points starting from hinge (y=0) extending to y=-span."""
    points = []
    for i in range(n + 1):
        t = i / n
        angle = math.pi * t
        y = -span * t
        z = rise * math.sin(angle)
        points.append((0.0, y, z))
    return points


def _build_bow_mesh(half_span: float, rise: float) -> MeshGeometry:
    """Build a tubular arch bow centered at origin, spanning Y."""
    pts = _bow_arch_points(half_span, rise, n=12)
    return tube_from_spline_points(
        pts,
        radius=BOW_TUBE_R,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _build_hinged_bow_mesh(span: float, rise: float) -> MeshGeometry:
    """Build a tubular arch bow in local hinge coords (y=0 to y=-span)."""
    pts = _bow_arch_points_from_hinge(span, rise, n=12)
    return tube_from_spline_points(
        pts,
        radius=BOW_TUBE_R,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _build_tarp_mesh(
    x_positions: list[float],
    half_span: float,
    rise: float,
    thickness: float = TARP_THICKNESS,
    n_arch_pts: int = 14,
) -> MeshGeometry:
    """Build a thin curved tarp surface following arch profiles at given X positions."""
    geo = MeshGeometry()

    # Generate arch profile (y, z) points
    arch_yz = []
    for i in range(n_arch_pts + 1):
        t = i / n_arch_pts
        angle = math.pi * t
        y = -half_span + 2.0 * half_span * t
        z = rise * math.sin(angle)
        arch_yz.append((y, z))

    n_pts = len(arch_yz)
    n_x = len(x_positions)

    # Top surface vertices
    for xi, x in enumerate(x_positions):
        for yi, (y, z) in enumerate(arch_yz):
            geo.add_vertex(x, y, z)

    # Bottom surface vertices (offset downward for thickness)
    for xi, x in enumerate(x_positions):
        for yi, (y, z) in enumerate(arch_yz):
            geo.add_vertex(x, y, z - thickness)

    # Top surface triangles
    for xi in range(n_x - 1):
        for yi in range(n_pts - 1):
            i00 = xi * n_pts + yi
            i01 = xi * n_pts + yi + 1
            i10 = (xi + 1) * n_pts + yi
            i11 = (xi + 1) * n_pts + yi + 1
            geo.add_face(i00, i10, i01)
            geo.add_face(i01, i10, i11)

    # Bottom surface triangles (reversed winding for outward normals)
    offset = n_x * n_pts
    for xi in range(n_x - 1):
        for yi in range(n_pts - 1):
            i00 = offset + xi * n_pts + yi
            i01 = offset + xi * n_pts + yi + 1
            i10 = offset + (xi + 1) * n_pts + yi
            i11 = offset + (xi + 1) * n_pts + yi + 1
            geo.add_face(i00, i01, i10)
            geo.add_face(i01, i11, i10)

    # End caps (close the thin edges at first and last X)
    # Front end (first X): connect top and bottom edges
    for yi in range(n_pts - 1):
        t0 = yi
        t1 = yi + 1
        b0 = offset + yi
        b1 = offset + yi + 1
        geo.add_face(t0, t1, b0)
        geo.add_face(t1, b1, b0)

    # Rear end (last X): connect top and bottom edges
    last = n_x - 1
    for yi in range(n_pts - 1):
        t0 = last * n_pts + yi
        t1 = last * n_pts + yi + 1
        b0 = offset + last * n_pts + yi
        b1 = offset + last * n_pts + yi + 1
        geo.add_face(t0, b0, t1)
        geo.add_face(t1, b0, b1)

    return geo


def _build_hinged_tarp_mesh(
    x_half_span: float,
    bow_span: float,
    rise: float,
    thickness: float = TARP_THICKNESS,
    n_arch_pts: int = 14,
) -> MeshGeometry:
    """Build tarp section for hinged bow in local coords (y=0 to y=-bow_span)."""
    geo = MeshGeometry()

    # Arch points from hinge (y=0) to far side (y=-bow_span)
    arch_yz = []
    for i in range(n_arch_pts + 1):
        t = i / n_arch_pts
        angle = math.pi * t
        y = -bow_span * t
        z = rise * math.sin(angle)
        arch_yz.append((y, z))

    n_pts = len(arch_yz)
    x_positions = [-x_half_span, x_half_span]
    n_x = len(x_positions)

    # Top surface
    for xi, x in enumerate(x_positions):
        for yi, (y, z) in enumerate(arch_yz):
            geo.add_vertex(x, y, z)

    # Bottom surface
    for xi, x in enumerate(x_positions):
        for yi, (y, z) in enumerate(arch_yz):
            geo.add_vertex(x, y, z - thickness)

    # Top triangles
    for xi in range(n_x - 1):
        for yi in range(n_pts - 1):
            i00 = xi * n_pts + yi
            i01 = xi * n_pts + yi + 1
            i10 = (xi + 1) * n_pts + yi
            i11 = (xi + 1) * n_pts + yi + 1
            geo.add_face(i00, i10, i01)
            geo.add_face(i01, i10, i11)

    # Bottom triangles
    offset = n_x * n_pts
    for xi in range(n_x - 1):
        for yi in range(n_pts - 1):
            i00 = offset + xi * n_pts + yi
            i01 = offset + xi * n_pts + yi + 1
            i10 = offset + (xi + 1) * n_pts + yi
            i11 = offset + (xi + 1) * n_pts + yi + 1
            geo.add_face(i00, i01, i10)
            geo.add_face(i01, i11, i10)

    # End caps
    for yi in range(n_pts - 1):
        t0 = yi
        t1 = yi + 1
        b0 = offset + yi
        b1 = offset + yi + 1
        geo.add_face(t0, t1, b0)
        geo.add_face(t1, b1, b0)
        t0r = (n_x - 1) * n_pts + yi
        t1r = (n_x - 1) * n_pts + yi + 1
        b0r = offset + (n_x - 1) * n_pts + yi
        b1r = offset + (n_x - 1) * n_pts + yi + 1
        geo.add_face(t0r, b0r, t1r)
        geo.add_face(t1r, b0r, b1r)

    return geo


def _build_hinged_tarp_mesh_asymmetric(
    x_fwd: float,
    x_back: float,
    bow_span: float,
    rise: float,
    thickness: float = TARP_THICKNESS,
    n_arch_pts: int = 14,
) -> MeshGeometry:
    """Build tarp section with asymmetric X extents (forward/backward from bow)."""
    geo = MeshGeometry()

    # Arch points from hinge (y=0) to far side (y=-bow_span)
    arch_yz = []
    for i in range(n_arch_pts + 1):
        t = i / n_arch_pts
        angle = math.pi * t
        y = -bow_span * t
        z = rise * math.sin(angle)
        arch_yz.append((y, z))

    n_pts = len(arch_yz)
    # Forward is +X, backward is -X
    x_positions = [x_fwd, -x_back]
    n_x = len(x_positions)

    # Top surface
    for xi, x in enumerate(x_positions):
        for yi, (y, z) in enumerate(arch_yz):
            geo.add_vertex(x, y, z)

    # Bottom surface
    for xi, x in enumerate(x_positions):
        for yi, (y, z) in enumerate(arch_yz):
            geo.add_vertex(x, y, z - thickness)

    # Top triangles
    for xi in range(n_x - 1):
        for yi in range(n_pts - 1):
            i00 = xi * n_pts + yi
            i01 = xi * n_pts + yi + 1
            i10 = (xi + 1) * n_pts + yi
            i11 = (xi + 1) * n_pts + yi + 1
            geo.add_face(i00, i10, i01)
            geo.add_face(i01, i10, i11)

    # Bottom triangles
    offset = n_x * n_pts
    for xi in range(n_x - 1):
        for yi in range(n_pts - 1):
            i00 = offset + xi * n_pts + yi
            i01 = offset + xi * n_pts + yi + 1
            i10 = offset + (xi + 1) * n_pts + yi
            i11 = offset + (xi + 1) * n_pts + yi + 1
            geo.add_face(i00, i01, i10)
            geo.add_face(i01, i11, i10)

    # End caps
    for yi in range(n_pts - 1):
        t0 = yi
        t1 = yi + 1
        b0 = offset + yi
        b1 = offset + yi + 1
        geo.add_face(t0, t1, b0)
        geo.add_face(t1, b1, b0)
        t0r = (n_x - 1) * n_pts + yi
        t1r = (n_x - 1) * n_pts + yi + 1
        b0r = offset + (n_x - 1) * n_pts + yi
        b1r = offset + (n_x - 1) * n_pts + yi + 1
        geo.add_face(t0r, b0r, t1r)
        geo.add_face(t1r, b0r, b1r)

    return geo


# =====================================================================
# Model construction
# =====================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_top_shipping_container")

    white = model.material("white_steel", rgba=(0.90, 0.91, 0.92, 1.0))
    grey = model.material("grey_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("dark_hardware", rgba=(0.18, 0.19, 0.21, 1.0))
    tarp = model.material("tarp_canvas", rgba=(0.12, 0.22, 0.38, 1.0))

    # =================================================================
    # BODY (root): corrugated box open at +X end and open on top.
    # Floor, front end wall, two side walls, top rails, corner castings,
    # fixed bows, and main tarp cover. NO fixed roof panel.
    # =================================================================
    body = model.part("body")

    # --- floor (steel plate) ---
    floor = BoxGeometry((body_len, W, 0.10))
    floor.translate(body_cx, 0.0, 0.05)
    body.visual(mesh_from_geometry(floor, "floor"), material=grey, name="floor")

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

    # --- door header & sill rails at the +X opening ---
    header = BoxGeometry((0.10, W, 0.14))
    header.translate(BODY_X1 - 0.05, 0.0, H - 0.07)
    body.visual(mesh_from_geometry(header, "door_header"), material=grey, name="door_header")
    sill = BoxGeometry((0.10, W, 0.14))
    sill.translate(BODY_X1 - 0.05, 0.0, 0.07)
    body.visual(mesh_from_geometry(sill, "door_sill"), material=grey, name="door_sill")
    # vertical corner posts at the doorway
    for sy, nm in ((1.0, "post_p"), (-1.0, "post_n")):
        post = BoxGeometry((0.12, 0.14, H))
        post.translate(BODY_X1 - 0.05, sy * (W / 2.0 - 0.07), H / 2.0)
        body.visual(mesh_from_geometry(post, nm), material=grey, name=nm)

    # --- 8 corner castings ---
    ci = 0
    for sx in (BODY_X0 + CORNER / 2.0, L / 2.0 - CORNER / 2.0):
        for sy in (-1.0, 1.0):
            for sz in (CORNER / 2.0, H - CORNER / 2.0):
                cc = _corner_casting()
                cc.translate(sx, sy * (W / 2.0 - CORNER / 2.0), sz)
                body.visual(mesh_from_geometry(cc, f"corner_{ci}"), material=dark, name=f"corner_{ci}")
                ci += 1

    # --- top rails (longitudinal steel channels on top of each side wall) ---
    for sy, nm in ((1.0, "top_rail_p"), (-1.0, "top_rail_n")):
        rail = BoxGeometry((body_len, TOP_RAIL_W, TOP_RAIL_H))
        rail.translate(body_cx, sy * (W / 2.0 - TOP_RAIL_W / 2.0), H - TOP_RAIL_H / 2.0)
        body.visual(mesh_from_geometry(rail, nm), material=grey, name=nm)

    # --- fixed bows (4 tubular steel arches, indexed 0..3) ---
    bow_half_span = BOW_SPAN / 2.0
    for i in range(N_BOWS - 1):
        bow_mesh = _build_bow_mesh(bow_half_span, BOW_RISE)
        bow_mesh.translate(BOW_POSITIONS_X[i], 0.0, BOW_BASE_Z)
        body.visual(
            mesh_from_geometry(bow_mesh, f"bow_{i}"),
            material=grey,
            name=f"bow_{i}",
        )
        # Small mounting bracket at each bow base (both sides)
        for sy, bracket_nm in ((1.0, f"bracket_{i}_p"), (-1.0, f"bracket_{i}_n")):
            bracket = BoxGeometry((0.06, 0.06, 0.04))
            bracket.translate(BOW_POSITIONS_X[i], sy * bow_half_span, BOW_BASE_Z - 0.02)
            body.visual(mesh_from_geometry(bracket, bracket_nm), material=dark, name=bracket_nm)

    # --- main tarp cover (over fixed bows, body visual) ---
    # Tarp spans from bow_0 to midpoint between bow_3 and bow_4 position
    tarp_x_end = (BOW_POSITIONS_X[N_BOWS - 2] + BOW_POSITIONS_X[N_BOWS - 1]) / 2.0
    tarp_x_start = BOW_POSITIONS_X[0] - 0.10
    # Generate X sample points for smooth tarp surface
    n_tarp_x = 16
    tarp_xs = [tarp_x_start + j * (tarp_x_end - tarp_x_start) / (n_tarp_x - 1) for j in range(n_tarp_x)]
    main_tarp = _build_tarp_mesh(tarp_xs, bow_half_span, BOW_RISE)
    main_tarp.translate(0.0, 0.0, BOW_BASE_Z)
    body.visual(mesh_from_geometry(main_tarp, "tarp_main"), material=tarp, name="tarp_main")

    body.inertial = Inertial.from_geometry(
        Box((L, W, H)), mass=2200.0, origin=Origin(xyz=(0.0, 0.0, H / 2.0))
    )

    # =================================================================
    # DOORS: identical to parent
    # =================================================================
    door_half_w = (W - 0.16) / 2.0
    door_h = H - 0.40

    door_specs = [
        ("door_l", 1.0),
        ("door_r", -1.0),
    ]

    for dname, hinge_sign in door_specs:
        door = model.part(dname)
        panel = _corrugated_door_panel(door_half_w, door_h)
        panel.translate(0.0, -hinge_sign * (door_half_w / 2.0), 0.0)
        door.visual(mesh_from_geometry(panel, "door_panel"), material=white, name="door_panel")

        door.inertial = Inertial.from_geometry(
            Box((DOOR_T, door_half_w, door_h)),
            mass=80.0,
            origin=Origin(xyz=(0.0, -hinge_sign * door_half_w / 2.0, 0.0)),
        )

        hinge_y = hinge_sign * (door_half_w + 0.04)
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

        # ---- locking rods + cam handles ----
        free_dir = -hinge_sign
        rod_ys = [
            free_dir * (door_half_w * 0.30),
            free_dir * (door_half_w * 0.78),
        ]
        rod_x = DOOR_T * 0.225 + CORR_DEPTH - 0.012
        for ridx, ry in enumerate(rod_ys):
            rod = CylinderGeometry(0.018, door_h - 0.10, radial_segments=16)
            rod.translate(rod_x, ry, 0.0)
            door.visual(mesh_from_geometry(rod, f"rod_{ridx}"), material=dark, name=f"rod_{ridx}")
            for sz in (1.0, -1.0):
                kp = BoxGeometry((0.05, 0.06, 0.05))
                kp.translate(rod_x - 0.02, ry, sz * (door_h / 2.0 - 0.18))
                door.visual(
                    mesh_from_geometry(kp, f"keeper_{ridx}_{0 if sz > 0 else 1}"),
                    material=grey,
                    name=f"keeper_{ridx}_{0 if sz > 0 else 1}",
                )

            hsuffix = ("l" if hinge_sign > 0 else "r") + str(ridx)
            hname = f"handle_{hsuffix}"
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
                origin=Origin(xyz=(rod_x, ry, 0.05)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=20.0, velocity=3.0, lower=0.0, upper=math.radians(110.0)
                ),
            )

    # =================================================================
    # HINGED BOW: one bow (at the last position) hinged at +Y top rail
    # =================================================================
    bow_hinged = model.part("bow_hinged")

    # Bow arch tube in local coords (hinge at origin, extends -Y)
    hinged_bow_mesh = _build_hinged_bow_mesh(BOW_SPAN, BOW_RISE)
    bow_hinged.visual(
        mesh_from_geometry(hinged_bow_mesh, "bow_arch"),
        material=grey,
        name="bow_arch",
    )

    # Tarp section on hinged bow: extends backward toward the previous bow only,
    # with minimal forward extension (not past the corner castings)
    hinged_tarp_back_x = _bow_spacing / 2.0  # backward span
    hinged_tarp_fwd_x = 0.08  # minimal forward span to stay clear of door hardware
    hinged_tarp = _build_hinged_tarp_mesh_asymmetric(
        hinged_tarp_fwd_x, hinged_tarp_back_x, BOW_SPAN, BOW_RISE
    )
    hinged_tarp.translate(0.0, 0.0, 0.0)  # already in local coords
    bow_hinged.visual(
        mesh_from_geometry(hinged_tarp, "tarp_hinged"),
        material=tarp,
        name="tarp_hinged",
    )

    # Mounting bracket at the non-hinge (-Y) end
    bracket_far = BoxGeometry((0.06, 0.06, 0.04))
    bracket_far.translate(0.0, -BOW_SPAN, -0.02)
    bow_hinged.visual(
        mesh_from_geometry(bracket_far, "bracket_far"),
        material=dark,
        name="bracket_far",
    )

    # Hinge pin barrel at the +Y base (on the bow part, rotates between ear brackets)
    # Length spans between the two ear brackets
    hinge_bracket = CylinderGeometry(0.022, 0.14, radial_segments=12)
    hinge_bracket.rotate_y(math.pi / 2.0)  # align along X
    bow_hinged.visual(
        mesh_from_geometry(hinge_bracket, "hinge_barrel"),
        material=dark,
        name="hinge_barrel",
    )

    bow_hinged.inertial = Inertial.from_geometry(
        Box((0.10, BOW_SPAN, BOW_RISE + 0.05)),
        mass=15.0,
        origin=Origin(xyz=(0.0, -BOW_SPAN / 2.0, BOW_RISE / 2.0)),
    )

    # Articulation: hinged at +Y top rail, axis along +X
    # At hinge point, bow extends in -Y. For positive q to lift the -Y end up:
    # axis = (-1, 0, 0) -- right-hand rule makes -Y rotate toward +Z
    hinge_x = BOW_POSITIONS_X[N_BOWS - 1]
    hinge_y = BOW_SPAN / 2.0  # +Y side (bow_half_span from center)
    model.articulation(
        "body_to_bow_hinged",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bow_hinged,
        origin=Origin(xyz=(hinge_x, hinge_y, BOW_BASE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=1.5, lower=0.0, upper=math.radians(80.0)
        ),
    )

    # Hinge bracket pair on the body side: two ears flanking the hinge barrel
    # Ear inner faces at hinge_x ± 0.05, ear width 0.06
    for x_off, nm in ((0.05, "hinge_ear_0"), (-0.05, "hinge_ear_1")):
        ear = BoxGeometry((0.06, 0.06, 0.08))
        ear.translate(hinge_x + x_off, hinge_y, BOW_BASE_Z - 0.04)
        body.visual(
            mesh_from_geometry(ear, nm),
            material=dark,
            name=nm,
        )

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
    bow_hinged = object_model.get_part("bow_hinged")
    bow_hinge = object_model.get_articulation("body_to_bow_hinged")

    # ---- container is a long box: length(X) >> width(Y), and tall ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "container is a long box (X longest, X>Y, X>Z)",
        body_ext[0] > body_ext[1] + 1.0 and body_ext[0] > body_ext[2] + 1.0,
        details=f"body extents={body_ext}",
    )

    # ---- no fixed roof: body top should be open (no visual named 'roof') ----
    body_visuals = [v.name for v in body.visuals]
    ctx.check(
        "no fixed roof panel (open top)",
        "roof" not in body_visuals,
        details=f"body visuals include: {body_visuals}",
    )

    # ---- fixed bows exist on the body (4 arches) ----
    for i in range(N_BOWS - 1):
        bow_name = f"bow_{i}"
        ctx.check(
            f"fixed bow {bow_name} exists on body",
            bow_name in body_visuals,
            details=f"looking for {bow_name} in body visuals",
        )

    # ---- bows span the container width ----
    for i in range(N_BOWS - 1):
        bow_visual = body.get_visual(f"bow_{i}")
        if bow_visual is not None:
            bow_dims = ctx.part_element_world_aabb(body, elem=f"bow_{i}")
            if bow_dims is not None:
                mn, mx = bow_dims
                bow_y_extent = mx[1] - mn[1]
                ctx.check(
                    f"bow_{i} spans the container width (Y extent > {BOW_SPAN * 0.8:.2f}m)",
                    bow_y_extent > BOW_SPAN * 0.8,
                    details=f"bow_{i} Y extent={bow_y_extent:.3f}m",
                )

    # ---- bows are arched (Z extent shows the rise) ----
    for i in range(min(2, N_BOWS - 1)):
        bow_dims = ctx.part_element_world_aabb(body, elem=f"bow_{i}")
        if bow_dims is not None:
            mn, mx = bow_dims
            bow_z_extent = mx[2] - mn[2]
            ctx.check(
                f"bow_{i} is arched (Z extent shows rise > {BOW_RISE * 0.5:.2f}m)",
                bow_z_extent > BOW_RISE * 0.5,
                details=f"bow_{i} Z extent={bow_z_extent:.3f}m",
            )

    # ---- hinged bow exists and is articulated ----
    ctx.check(
        "hinged bow part exists",
        bow_hinged is not None,
        details="bow_hinged part not found",
    )

    # ---- tarp cover exists ----
    ctx.check(
        "main tarp cover exists on body",
        "tarp_main" in body_visuals,
        details="tarp_main visual not found on body",
    )

    # ---- doors are at the +X (cargo) end ----
    for d, nm in ((door_l, "door_l"), (door_r, "door_r")):
        pos = ctx.part_world_position(d)
        ctx.check(
            f"{nm} is at the +X cargo end",
            pos is not None and pos[0] > L / 2.0 - 0.3,
            details=f"{nm} origin={pos}",
        )

    # ---- doors hinge on doorway corner posts ----
    ctx.allow_overlap(
        door_l, body,
        elem_a="door_panel", elem_b="post_p",
        reason="Left door's hinge edge frame seats against the +Y doorway corner post.",
    )
    ctx.allow_overlap(
        door_r, body,
        elem_a="door_panel", elem_b="post_n",
        reason="Right door's hinge edge frame seats against the -Y doorway corner post.",
    )
    ctx.expect_contact(door_l, body, name="left door seated against doorway")
    ctx.expect_contact(door_r, body, name="right door seated against doorway")

    # ---- cam handles seated on rods ----
    for hsuffix, ridx in (("l0", 0), ("l1", 1), ("r0", 0), ("r1", 1)):
        hname = f"handle_{hsuffix}"
        dname = "door_l" if hsuffix.startswith("l") else "door_r"
        d = object_model.get_part(dname)
        h = object_model.get_part(hname)
        ctx.allow_overlap(
            d, h, elem_a=f"rod_{ridx}", elem_b="hub",
            reason="Cam-handle hub seats around the locking rod it actuates.",
        )
        ctx.allow_overlap(
            d, h, elem_a=f"rod_{ridx}", elem_b="lever",
            reason="Cam lever passes over its locking rod where it clamps.",
        )
        ctx.allow_overlap(
            d, h, elem_a="door_panel", elem_b="hub",
            reason="Handle hub sits flush against the corrugated door face.",
        )

    # ---- both doors swing open ----
    for d, hinge, nm in ((door_l, hinge_l, "door_l"), (door_r, hinge_r, "door_r")):
        x0 = _ext(ctx.part_world_aabb(d))[0]
        with ctx.pose({hinge: math.radians(110.0)}):
            x1 = _ext(ctx.part_world_aabb(d))[0]
        ctx.check(
            f"{nm} swings open (free edge moves outward in +X)",
            x1 > x0 + 0.4,
            details=f"{nm} X-extent rest={x0:.3f} open={x1:.3f}",
        )

    # ---- cam handles rotate about their rod axis ----
    for hsuffix in ("l0", "l1", "r0", "r1"):
        hname = f"handle_{hsuffix}"
        dname = "door_l" if hsuffix.startswith("l") else "door_r"
        handle = object_model.get_part(hname)
        joint = object_model.get_articulation(f"{dname}_to_{hname}")
        ctx.expect_contact(handle, object_model.get_part(dname), name=f"{hname} mounted on its door rod")
        ext0 = _ext(ctx.part_world_aabb(handle))
        with ctx.pose({joint: math.radians(90.0)}):
            ext90 = _ext(ctx.part_world_aabb(handle))
        ctx.check(
            f"{hname} rotates about its rod axis (lever sweeps)",
            abs(ext90[1] - ext0[1]) > 0.05 or abs(ext90[0] - ext0[0]) > 0.05,
            details=f"{hname} rest_ext={ext0} turned_ext={ext90}",
        )

    # ---- hinged bow opens: free end (-Y side) rises in +Z ----
    bow_pos_rest = ctx.part_world_position(bow_hinged)
    with ctx.pose({bow_hinge: math.radians(70.0)}):
        bow_pos_open = ctx.part_world_position(bow_hinged)
        bow_aabb_open = ctx.part_world_aabb(bow_hinged)
    # The free end should rise significantly
    bow_aabb_rest = ctx.part_world_aabb(bow_hinged)
    # Check that the bow part's max Z increases when opened
    ctx.check(
        "hinged bow opens upward (max Z increases)",
        bow_aabb_open is not None and bow_aabb_rest is not None
        and bow_aabb_open[1][2] > bow_aabb_rest[1][2] + 0.10,
        details=f"rest max_z={bow_aabb_rest[1][2]:.3f} open max_z={bow_aabb_open[1][2]:.3f}",
    )

    # ---- hinged bow: hinge barrel captured between ear brackets on the body ----
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="hinge_barrel", elem_b="hinge_ear_0",
        reason="Hinge barrel is captured between the two ear brackets on the body top rail.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="hinge_barrel", elem_b="hinge_ear_1",
        reason="Hinge barrel is captured between the two ear brackets on the body top rail.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="bracket_far", elem_b="top_rail_n",
        reason="Far-end mounting bracket seats on the -Y top rail to support the bow when closed.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="tarp_hinged", elem_b="door_header",
        reason="Hinged tarp extends to the door end and seats against the door header at the +X opening.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="tarp_hinged", elem_b="hinge_ear_0",
        reason="Tarp attaches at the hinge axis where the ear bracket supports the hinge barrel.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="tarp_hinged", elem_b="hinge_ear_1",
        reason="Tarp attaches at the hinge axis where the ear bracket supports the hinge barrel.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="bow_arch", elem_b="hinge_ear_0",
        reason="Bow arch tube starts at the hinge axis where the ear bracket is mounted.",
    )
    ctx.allow_overlap(
        bow_hinged, body,
        elem_a="bow_arch", elem_b="hinge_ear_1",
        reason="Bow arch tube starts at the hinge axis where the ear bracket is mounted.",
    )
    # Prove the hinge mechanism works: barrel X extent overlaps ear X extent
    ctx.expect_overlap(
        bow_hinged, body,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="hinge_ear_0",
        min_overlap=0.01,
        name="hinge barrel overlaps ear_0 in X",
    )
    ctx.expect_overlap(
        bow_hinged, body,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="hinge_ear_1",
        min_overlap=0.01,
        name="hinge barrel overlaps ear_1 in X",
    )

    return ctx.report()


object_model = build_object_model()
