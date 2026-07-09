from __future__ import annotations

# Public street trash can: a tall cylindrical drum with a front swing-door
# disposal hatch. The upper front of the drum carries a curved rectangular
# opening closed by a hinged door that swings outward and up about a
# horizontal hinge along its top edge.
#
# Coordinate convention:
#   +Z is up; the drum floor sits at z=0.
#   +X is the front of the can (opening faces +X).
#   The hinge axis is along Y (horizontal, across the opening top).
#
# Root structure: the drum is the root. The cap, banding hoops, and hinge
# knuckles are inline visuals on the drum. The door is a separate part
# connected by a single REVOLUTE articulation.
#
# Articulation:
#   drum_to_door : REVOLUTE, axis Y, positive q opens door outward.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
DRUM_R = 0.225            # drum outer radius (0.45 m diameter)
DRUM_H = 0.88             # drum height
WALL_T = 0.003            # wall thickness
FLOOR_T = 0.005           # floor slab thickness

HALF_ANGLE = 0.45         # half-angle of front opening (~25.8 deg)
OPENING_H = 0.20          # opening height
OPENING_BOT = 0.58        # opening bottom z
OPENING_TOP = OPENING_BOT + OPENING_H  # 0.78

CAP_T = 0.008             # cap disc thickness
CAP_OVERHANG = 0.005      # cap overhang past drum radius

DOOR_T = 0.004            # door panel thickness
DOOR_CLEAR = 0.003        # door edge clearance on each side

N_THETA = 72              # circumferential segments for drum
N_DOOR_T = 24             # circumferential segments for door


# ---- helpers -----------------------------------------------------------------

def _make_thetas(n_base: int, half_angle: float) -> list[float]:
    """Generate sorted theta samples for a full circle with exact opening
    boundaries inserted so the opening edge is clean."""
    thetas: list[float] = []
    for i in range(n_base):
        thetas.append(2.0 * math.pi * i / n_base)
    for a in (half_angle, 2.0 * math.pi - half_angle):
        if not any(abs(t - a) < 1e-6 for t in thetas):
            thetas.append(a)
    thetas.sort()
    return thetas


def _face_in_opening(thetas: list[float], ti: int, n: int,
                     half_angle: float) -> bool:
    """Return True when the face midpoint angle is inside the opening arc."""
    ti_next = (ti + 1) % n
    t1 = thetas[ti]
    t2 = thetas[ti_next]
    if t2 >= t1:
        tmid = (t1 + t2) / 2.0
    else:
        tmid = (t1 + t2 + 2.0 * math.pi) / 2.0
        if tmid >= 2.0 * math.pi:
            tmid -= 2.0 * math.pi
    return tmid < half_angle or tmid > 2.0 * math.pi - half_angle


# ---- drum shell mesh ---------------------------------------------------------

def _drum_shell() -> MeshGeometry:
    """Hollow cylindrical drum with a front rectangular opening.

    The mesh has outer wall, inner wall, bottom ring, top ring, interior
    floor disc, and four opening-throat walls connecting outer to inner
    at the cut edges. The interior is genuinely hollow.
    """
    geo = MeshGeometry()
    thetas = _make_thetas(N_THETA, HALF_ANGLE)
    N = len(thetas)

    # z-levels: [floor_base, floor_top, opening_bot, opening_top, drum_top]
    zs = [0.0, FLOOR_T, OPENING_BOT, OPENING_TOP, DRUM_H]
    NZ = len(zs)
    OPEN_BAND = 2  # between zs[2] and zs[3]

    # --- build outer and inner vertex grids ---
    ov: list[list[int]] = []
    iv: list[list[int]] = []
    for zi in range(NZ):
        z = zs[zi]
        o_row: list[int] = []
        i_row: list[int] = []
        for ti in range(N):
            th = thetas[ti]
            cx, cy = math.cos(th), math.sin(th)
            o_row.append(geo.add_vertex(DRUM_R * cx, DRUM_R * cy, z))
            i_row.append(geo.add_vertex(
                (DRUM_R - WALL_T) * cx, (DRUM_R - WALL_T) * cy, z))
        ov.append(o_row)
        iv.append(i_row)

    # --- outer wall faces (outward normals) ---
    for zi in range(NZ - 1):
        for ti in range(N):
            if zi == OPEN_BAND and _face_in_opening(
                    thetas, ti, N, HALF_ANGLE):
                continue
            ti_n = (ti + 1) % N
            a, b = ov[zi][ti], ov[zi][ti_n]
            c, d = ov[zi + 1][ti_n], ov[zi + 1][ti]
            geo.add_face(a, b, c)
            geo.add_face(a, c, d)

    # --- inner wall faces (inward normals) ---
    for zi in range(NZ - 1):
        for ti in range(N):
            if zi == OPEN_BAND and _face_in_opening(
                    thetas, ti, N, HALF_ANGLE):
                continue
            ti_n = (ti + 1) % N
            a, b = iv[zi][ti], iv[zi][ti_n]
            c, d = iv[zi + 1][ti_n], iv[zi + 1][ti]
            geo.add_face(a, d, c)
            geo.add_face(a, c, b)

    # --- bottom annular ring at z=0 (normals down) ---
    for ti in range(N):
        ti_n = (ti + 1) % N
        a, b = ov[0][ti], ov[0][ti_n]
        c, d = iv[0][ti_n], iv[0][ti]
        geo.add_face(a, d, c)
        geo.add_face(a, c, b)

    # --- top annular ring at z=DRUM_H (normals up) ---
    top = NZ - 1
    for ti in range(N):
        ti_n = (ti + 1) % N
        a, b = ov[top][ti], ov[top][ti_n]
        c, d = iv[top][ti_n], iv[top][ti]
        geo.add_face(a, b, c)
        geo.add_face(a, c, d)

    # --- floor disc at z=FLOOR_T (normals up, from center to inner wall) ---
    flr = 1  # zs[1] = FLOOR_T
    fc = geo.add_vertex(0.0, 0.0, FLOOR_T)
    for ti in range(N):
        ti_n = (ti + 1) % N
        geo.add_face(fc, iv[flr][ti_n], iv[flr][ti])

    # --- opening throat walls ---
    # Find theta indices at exact opening boundaries
    hi_idx = lo_idx = None
    for ti in range(N):
        if abs(thetas[ti] - HALF_ANGLE) < 1e-6:
            hi_idx = ti
        if abs(thetas[ti] - (2.0 * math.pi - HALF_ANGLE)) < 1e-6:
            lo_idx = ti

    if hi_idx is not None and lo_idx is not None:
        zb, zt = 2, 3  # OPENING_BOT and OPENING_TOP z-indices

        # Side wall at theta = HALF_ANGLE (right side of opening)
        ao, bo = ov[zb][hi_idx], ov[zt][hi_idx]
        ai, bi = iv[zb][hi_idx], iv[zt][hi_idx]
        geo.add_face(ao, ai, bi)
        geo.add_face(ao, bi, bo)

        # Side wall at theta = 2pi - HALF_ANGLE (left side of opening)
        ao, bo = ov[zb][lo_idx], ov[zt][lo_idx]
        ai, bi = iv[zb][lo_idx], iv[zt][lo_idx]
        geo.add_face(ao, bo, bi)
        geo.add_face(ao, bi, ai)

        # Opening arc indices (from left boundary through 0 to right boundary)
        opening_idx = list(range(lo_idx, N)) + list(range(0, hi_idx + 1))

        # Bottom throat wall at z = OPENING_BOT (normals down-ish / inward)
        for k in range(len(opening_idx) - 1):
            ti = opening_idx[k]
            ti_n = opening_idx[k + 1]
            ao, bo = ov[zb][ti], ov[zb][ti_n]
            ai, bi = iv[zb][ti], iv[zb][ti_n]
            geo.add_face(ao, bo, bi)
            geo.add_face(ao, bi, ai)

        # Top throat wall at z = OPENING_TOP (normals up-ish / inward)
        for k in range(len(opening_idx) - 1):
            ti = opening_idx[k]
            ti_n = opening_idx[k + 1]
            ao, bo = ov[zt][ti], ov[zt][ti_n]
            ai, bi = iv[zt][ti], iv[zt][ti_n]
            geo.add_face(ao, bi, bo)
            geo.add_face(ao, ai, bi)

    return geo


# ---- door panel mesh ---------------------------------------------------------

def _door_mesh() -> MeshGeometry:
    """Curved panel matching the drum curvature with inner and outer skins
    and closed edges. Authored in the door local frame where the hinge is
    at the origin and the panel hangs downward.
    """
    geo = MeshGeometry()

    door_ha = HALF_ANGLE - DOOR_CLEAR / DRUM_R  # door half-angle
    z_top = -DOOR_CLEAR
    z_bot = -(OPENING_H - DOOR_CLEAR)

    # theta samples from -door_ha to +door_ha
    dth: list[float] = []
    for i in range(N_DOOR_T + 1):
        dth.append(-door_ha + 2.0 * door_ha * i / N_DOOR_T)

    nz = 4
    dzs: list[float] = []
    for j in range(nz + 1):
        dzs.append(z_bot + (z_top - z_bot) * j / nz)

    R_out = DRUM_R
    R_in = DRUM_R - DOOR_T

    # Outer surface grid: x = R*cos(th) - DRUM_R, y = R*sin(th)
    og: list[list[int]] = []
    ig: list[list[int]] = []
    for j in range(nz + 1):
        z = dzs[j]
        o_row: list[int] = []
        i_row: list[int] = []
        for i in range(N_DOOR_T + 1):
            th = dth[i]
            cx, cy = math.cos(th), math.sin(th)
            o_row.append(geo.add_vertex(R_out * cx - DRUM_R, R_out * cy, z))
            i_row.append(geo.add_vertex(R_in * cx - DRUM_R, R_in * cy, z))
        og.append(o_row)
        ig.append(i_row)

    # Outer faces (normals away from drum axis)
    for j in range(nz):
        for i in range(N_DOOR_T):
            a, b = og[j][i], og[j][i + 1]
            c, d = og[j + 1][i + 1], og[j + 1][i]
            geo.add_face(a, b, c)
            geo.add_face(a, c, d)

    # Inner faces (normals toward drum axis)
    for j in range(nz):
        for i in range(N_DOOR_T):
            a, b = ig[j][i], ig[j][i + 1]
            c, d = ig[j + 1][i + 1], ig[j + 1][i]
            geo.add_face(a, d, c)
            geo.add_face(a, c, b)

    # Top edge skin
    tj = nz
    for i in range(N_DOOR_T):
        ao, bo = og[tj][i], og[tj][i + 1]
        ai, bi = ig[tj][i], ig[tj][i + 1]
        geo.add_face(ao, bo, bi)
        geo.add_face(ao, bi, ai)

    # Bottom edge skin
    for i in range(N_DOOR_T):
        ao, bo = og[0][i], og[0][i + 1]
        ai, bi = ig[0][i], ig[0][i + 1]
        geo.add_face(ao, ai, bi)
        geo.add_face(ao, bi, bo)

    # Left edge skin (i=0)
    for j in range(nz):
        ao, bo = og[j][0], og[j + 1][0]
        ai, bi = ig[j][0], ig[j + 1][0]
        geo.add_face(ao, bo, bi)
        geo.add_face(ao, bi, ai)

    # Right edge skin (i=N_DOOR_T)
    for j in range(nz):
        ao, bo = og[j][N_DOOR_T], og[j + 1][N_DOOR_T]
        ai, bi = ig[j][N_DOOR_T], ig[j + 1][N_DOOR_T]
        geo.add_face(ao, ai, bi)
        geo.add_face(ao, bi, bo)

    return geo


# ---- shared geometry helpers for repeated sub-parts --------------------------

def _make_hoop(radius: float, tube_r: float) -> MeshGeometry:
    """Banding hoop: a torus ring."""
    return TorusGeometry(
        radius, tube_r, radial_segments=12, tubular_segments=N_THETA
    )


def _make_knuckle(radius: float, length: float) -> MeshGeometry:
    """Hinge knuckle: a small cylinder oriented along Y."""
    cyl = CylinderGeometry(radius, length, radial_segments=12)
    cyl.rotate_x(-math.pi / 2.0)  # orient along +Y
    return cyl


# ---- build -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_trash_can")

    # Materials
    drum_mat = model.material(
        "drum_black", rgba=(0.11, 0.11, 0.12, 1.0))
    door_mat = model.material(
        "door_dark_gray", rgba=(0.16, 0.16, 0.17, 1.0))
    cap_mat = model.material(
        "cap_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    hoop_mat = model.material(
        "hoop_brushed", rgba=(0.52, 0.53, 0.55, 1.0))
    hinge_mat = model.material(
        "hinge_steel", rgba=(0.58, 0.60, 0.62, 1.0))

    # ---- drum body (root) ---------------------------------------------------
    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(_drum_shell(), "drum_shell"),
        material=drum_mat,
        name="drum_shell",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=DRUM_R, length=DRUM_H),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, DRUM_H * 0.45)),
    )

    # Cap: flat disc on top of the drum (inline visual, not a separate part)
    cap_geo = CylinderGeometry(
        DRUM_R + CAP_OVERHANG, CAP_T, radial_segments=64, closed=True
    )
    cap_geo.translate(0.0, 0.0, DRUM_H + CAP_T / 2.0)
    drum.visual(
        mesh_from_geometry(cap_geo, "cap_disc"),
        material=cap_mat,
        name="cap_disc",
    )

    # Banding hoops: repeated torus rings at several z-levels
    hoop_zs = [0.10, 0.42, DRUM_H - 0.05]
    for i in range(len(hoop_zs)):
        hoop = _make_hoop(DRUM_R + 0.002, 0.006)
        hoop.translate(0.0, 0.0, hoop_zs[i])
        drum.visual(
            mesh_from_geometry(hoop, f"hoop_{i}"),
            material=hoop_mat,
            name=f"hoop_{i}",
        )

    # Hinge knuckles: small cylinders along the top of the opening
    n_knuckles = 5
    knuckle_len = 0.022
    knuckle_r = 0.007
    hinge_span = 2.0 * DRUM_R * math.sin(HALF_ANGLE) * 0.80
    for i in range(n_knuckles):
        y_pos = -hinge_span / 2.0 + hinge_span * (i + 0.5) / n_knuckles
        kn = _make_knuckle(knuckle_r, knuckle_len)
        # Seat knuckle barrel center on the drum outer surface so it reads as
        # mounted and shares geometry contact with the shell.
        kn.translate(DRUM_R, y_pos, OPENING_TOP)
        drum.visual(
            mesh_from_geometry(kn, f"knuckle_{i}"),
            material=hinge_mat,
            name=f"knuckle_{i}",
        )

    # ---- door (swing hatch) -------------------------------------------------
    door = model.part("door")
    door.visual(
        mesh_from_geometry(_door_mesh(), "door_panel"),
        material=door_mat,
        name="door_panel",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_T + 0.01, 2.0 * DRUM_R * math.sin(HALF_ANGLE), OPENING_H)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, -OPENING_H / 2.0)),
    )

    # ---- articulation: drum_to_door (REVOLUTE) ------------------------------
    # Joint origin at the top of the opening on the drum front surface.
    # Axis (0, -1, 0): positive q opens the door outward (+X direction).
    model.articulation(
        "drum_to_door",
        ArticulationType.REVOLUTE,
        parent=drum,
        child=door,
        origin=Origin(xyz=(DRUM_R, 0.0, OPENING_TOP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=1.6,
        ),
    )

    return model


# ---- tests -------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("drum_to_door")

    # --- drum proportions ----------------------------------------------------
    daabb = ctx.part_world_aabb(drum)
    dext = _ext(daabb)
    ctx.check(
        "drum base sits at z=0",
        daabb is not None and abs(daabb[0][2]) < 0.008,
        details=f"drum_min_z={None if daabb is None else daabb[0][2]}",
    )
    ctx.check(
        "drum height ~0.88 m",
        abs(dext[2] - (DRUM_H + CAP_T)) < 0.03,
        details=f"drum_ext={dext}",
    )
    ctx.check(
        "drum diameter ~0.45 m",
        0.42 < dext[0] < 0.52 and 0.42 < dext[1] < 0.52,
        details=f"drum_ext={dext}",
    )

    # --- door is present and sized -------------------------------------------
    door_aabb = ctx.part_world_aabb(door)
    door_ext = _ext(door_aabb)
    ctx.check(
        "door height ~0.20 m",
        0.15 < door_ext[2] < 0.25,
        details=f"door_ext={door_ext}",
    )
    ctx.check(
        "door width ~0.19 m",
        0.12 < door_ext[1] < 0.26,
        details=f"door_ext={door_ext}",
    )
    ctx.check(
        "door is on the front (+X) of the drum",
        door_aabb[0][0] > 0.0,
        details=f"door_min_x={door_aabb[0][0]}",
    )

    # --- door joint is REVOLUTE with horizontal axis -------------------------
    ctx.check(
        "door joint is revolute",
        str(door_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={door_joint.articulation_type}",
    )
    ctx.check(
        "door hinge axis is lateral (Y)",
        abs(door_joint.axis[1]) > 0.99
        and abs(door_joint.axis[0]) < 0.01
        and abs(door_joint.axis[2]) < 0.01,
        details=f"axis={door_joint.axis}",
    )
    lim = door_joint.motion_limits
    ctx.check(
        "door travel past 80 degrees",
        lim is not None and lim.upper is not None and lim.upper >= 1.4,
        details=f"upper={lim.upper}",
    )

    # --- door opens outward --------------------------------------------------
    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.2}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door bottom swings outward (max_x increases)",
        open_aabb[1][0] > rest_aabb[1][0] + 0.08,
        details=(
            f"rest_max_x={rest_aabb[1][0]:.4f}, "
            f"open_max_x={open_aabb[1][0]:.4f}"
        ),
    )

    # --- opening is a real void: at full open, door is well away from drum ---
    with ctx.pose({door_joint: 1.5}):
        full_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "fully open door extends well past drum surface",
        full_open_aabb[1][0] > DRUM_R + 0.12,
        details=(
            f"door_max_x={full_open_aabb[1][0]:.4f}, "
            f"drum_r={DRUM_R}"
        ),
    )

    # --- door covers the opening when closed ---------------------------------
    ctx.expect_overlap(
        door, drum, axes="y",
        min_overlap=0.10,
        name="closed door overlaps drum footprint in Y (covers opening)",
    )
    ctx.expect_overlap(
        door, drum, axes="z",
        min_overlap=0.10,
        name="closed door overlaps drum footprint in Z (covers opening)",
    )

    return ctx.report()


object_model = build_object_model()
