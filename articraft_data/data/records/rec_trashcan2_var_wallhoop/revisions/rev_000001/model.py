from __future__ import annotations

# Small cylindrical black plastic swing-lid trash can, WALL-MOUNTED via a steel
# hoop bracket.  The can sits in a wall-fixed steel ring/hoop bracket that bolts
# flat to a vertical wall plate.  The can is held against the wall and stands
# off the ground with no legs of its own.
#
# Structure:
#   - wall_plate (root): flat steel plate with hoop ring, support arms, bolts
#   - body: tapered cylindrical shell, open top (FIXED child of wall_plate)
#   - lid: silver domed lid with teardrop opening (FIXED child of body)
#   - flap: teardrop swing flap (REVOLUTE child of lid, primary articulation)
#
# Coordinate convention:
#   +Z up, ground at z = 0
#   Wall surface at y = 0, can extends in +Y from the wall
#   Dome/flap axis along Z; flap rocker pivot is horizontal Y through flap center

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- body / lid / flap dimensions (meters) ----------------------------------
R_BOTTOM = 0.105            # body radius at the floor
R_TOP = 0.122               # body radius at the mouth (slight outward taper)
BODY_H = 0.250              # body height (mouth rim z in body-local frame)
WALL_T = 0.004
FLOOR_T = 0.008

LID_OVERHANG = 0.010
LID_R = R_TOP + LID_OVERHANG
LID_RISE = 0.060
LID_SKIRT = 0.022

FLAP_LEN = 0.130
FLAP_WID = 0.095
FLAP_RISE = 0.022

# ---- wall mount dimensions ---------------------------------------------------
MOUNT_Z = 0.550             # can bottom height above ground

PLATE_W = 0.180             # wall plate width  (X)
PLATE_H = 0.420             # wall plate height (Z)
PLATE_T = 0.006             # wall plate thickness (Y)

HOOP_R = 0.127              # hoop major radius (centre-of-tube to axis)
HOOP_TUBE = 0.008           # hoop tube cross-section radius

ARM_W = 0.022               # support arm width  (X)
ARM_H = 0.025               # support arm height (Z)
ARM_X = 0.025               # arm X offset from centre (±) – close enough to reach hoop
ARM_DEPTH = 0.075           # arm depth (Y) – long enough to meet hoop ring

BOLT_R = 0.006              # bolt-head radius
BOLT_H = 0.005              # bolt-head height (protrusion from plate face)

# ---- derived positions -------------------------------------------------------
CAN_Y = PLATE_T + ARM_DEPTH + HOOP_R   # can axis Y from wall  (~0.211)
HOOP_Z = MOUNT_Z + BODY_H * 0.85       # hoop ring height      (~0.763)
PLATE_Z_CTR = MOUNT_Z + BODY_H * 0.50  # plate centre height   (~0.675)

BOLT_X_OFF = PLATE_W * 0.5 - 0.020     # bolt X inset from plate edge
BOLT_Z_OFF = PLATE_H * 0.5 - 0.025     # bolt Z inset from plate edge


# =============================================================================
# geometry helpers – body / lid / flap  (kept from parent)
# =============================================================================

def _body_shell() -> MeshGeometry:
    """Hollow tapered cylindrical body, open top, floor on z=0."""
    outer = [
        (R_BOTTOM, 0.0),
        (R_BOTTOM + 0.004, 0.010),
        (R_TOP, BODY_H),
    ]
    inner = [
        (R_BOTTOM - WALL_T, FLOOR_T),
        (R_BOTTOM - WALL_T + 0.004, FLOOR_T + 0.010),
        (R_TOP - WALL_T, BODY_H),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer, inner, segments=64, start_cap="flat", end_cap="flat"
    )
    floor = CylinderGeometry(R_BOTTOM - WALL_T, FLOOR_T, radial_segments=64)
    floor.translate(0.0, 0.0, FLOOR_T / 2.0)
    shell.merge(floor)
    return shell


def _hole_r(theta: float) -> float:
    """Teardrop boundary radius at angle theta (long axis along X)."""
    a = FLAP_LEN / 2.0
    b = FLAP_WID / 2.0
    er = (a * b) / math.hypot(b * math.cos(theta), a * math.sin(theta))
    taper = 1.0 - 0.12 * max(0.0, -math.cos(theta))
    return er * taper


def _lid_mesh() -> MeshGeometry:
    """Silver domed lid with teardrop opening and sleeve skirt."""
    geo = MeshGeometry()
    rim_z = BODY_H
    cr = LID_R
    seg = 64
    rings = 10

    def dome_z(r):
        t = min(1.0, max(0.0, r / cr))
        return rim_z + LID_RISE * math.cos(t * math.pi / 2.0)

    grid_out: list[list[tuple[float, float, float]]] = []
    grid_in: list[list[tuple[float, float, float]]] = []
    for i in range(rings + 1):
        f = i / rings
        ring_out: list[tuple[float, float, float]] = []
        ring_in: list[tuple[float, float, float]] = []
        for j in range(seg):
            th = 2 * math.pi * j / seg
            hr = _hole_r(th)
            r = cr + (hr - cr) * f
            z = dome_z(r)
            ring_out.append((r * math.cos(th), r * math.sin(th), z))
            ring_in.append((r * math.cos(th), r * math.sin(th), z - 0.004))
        grid_out.append(ring_out)
        grid_in.append(ring_in)

    def add_quad_grid(grid, flip=False):
        idx = [[geo.add_vertex(*p) for p in ring] for ring in grid]
        for i in range(len(grid) - 1):
            for j in range(seg):
                jn = (j + 1) % seg
                a, b, c, d = idx[i][j], idx[i][jn], idx[i + 1][jn], idx[i + 1][j]
                if flip:
                    geo.add_face(a, c, b); geo.add_face(a, d, c)
                else:
                    geo.add_face(a, b, c); geo.add_face(a, c, d)
        return idx

    out_idx = add_quad_grid(grid_out, flip=False)
    in_idx = add_quad_grid(grid_in, flip=True)
    last = rings
    for j in range(seg):
        jn = (j + 1) % seg
        a, b = out_idx[last][j], out_idx[last][jn]
        c, d = in_idx[last][jn], in_idx[last][j]
        geo.add_face(a, b, c); geo.add_face(a, c, d)

    skirt = CylinderGeometry(cr, LID_SKIRT, radial_segments=seg, closed=False)
    skirt.translate(0.0, 0.0, rim_z - LID_SKIRT / 2.0)
    geo.merge(skirt)

    # ---- inner locating skirt: thin wall inside the body mouth that ----
    # ---- contacts the body outer wall and seats the lid securely.  ----
    inner_r = R_TOP - 0.001  # 0.121 – slightly inside body outer wall
    inner_h = 0.010
    inner_top_z = rim_z
    inner_bot_z = rim_z - inner_h

    inner_top_idx: list[int] = []
    inner_bot_idx: list[int] = []
    for j in range(seg):
        th = 2 * math.pi * j / seg
        ix = inner_r * math.cos(th)
        iy = inner_r * math.sin(th)
        inner_top_idx.append(geo.add_vertex(ix, iy, inner_top_z))
        inner_bot_idx.append(geo.add_vertex(ix, iy, inner_bot_z))

    # inner wall (normals face inward toward can centre)
    for j in range(seg):
        jn = (j + 1) % seg
        geo.add_face(inner_top_idx[j], inner_bot_idx[j], inner_bot_idx[jn])
        geo.add_face(inner_top_idx[j], inner_bot_idx[jn], inner_top_idx[jn])

    # annular bridge connecting dome underside ring-1 to inner skirt top
    for j in range(seg):
        jn = (j + 1) % seg
        d1 = in_idx[1][j]
        d1n = in_idx[1][jn]
        i1 = inner_top_idx[j]
        i1n = inner_top_idx[jn]
        geo.add_face(d1, i1, i1n)
        geo.add_face(d1, i1n, d1n)

    return geo


def _flap_mesh() -> MeshGeometry:
    """Teardrop curved swing flap centred on its own pivot."""
    geo = MeshGeometry()
    seg = 48
    rings = 6

    grid: list[list[tuple[float, float, float]]] = []
    for i in range(rings + 1):
        f = i / rings
        ring: list[tuple[float, float, float]] = []
        for j in range(seg):
            th = 2 * math.pi * j / seg
            er = _hole_r(th)
            r = er * f
            z = FLAP_RISE * math.cos(f * math.pi / 2.0)
            ring.append((r * math.cos(th), r * math.sin(th), z))
        grid.append(ring)
    idx = [[geo.add_vertex(*p) for p in ring] for ring in grid]
    center_top = idx[0][0]
    for j in range(seg):
        jn = (j + 1) % seg
        geo.add_face(center_top, idx[1][j], idx[1][jn])
    for i in range(1, rings):
        for j in range(seg):
            jn = (j + 1) % seg
            a0, b0, c0, d0 = idx[i][j], idx[i][jn], idx[i + 1][jn], idx[i + 1][j]
            geo.add_face(a0, b0, c0); geo.add_face(a0, c0, d0)

    base_idx = [[geo.add_vertex(grid[i][j][0], grid[i][j][1], 0.0)
                 for j in range(seg)] for i in range(rings + 1)]
    center_bot = geo.add_vertex(0.0, 0.0, 0.0)
    for j in range(seg):
        jn = (j + 1) % seg
        geo.add_face(center_bot, base_idx[1][jn], base_idx[1][j])
    for i in range(1, rings):
        for j in range(seg):
            jn = (j + 1) % seg
            a0, b0, c0, d0 = base_idx[i][j], base_idx[i][jn], base_idx[i + 1][jn], base_idx[i + 1][j]
            geo.add_face(a0, c0, b0); geo.add_face(a0, d0, c0)

    for j in range(seg):
        jn = (j + 1) % seg
        t, tn = idx[rings][j], idx[rings][jn]
        bse, bsen = base_idx[rings][j], base_idx[rings][jn]
        geo.add_face(t, tn, bsen); geo.add_face(t, bsen, bse)
    return geo


# =============================================================================
# geometry helpers – wall mount  (new, shared helpers used in for-i loops)
# =============================================================================

def _bolt_head_geo() -> MeshGeometry:
    """Hex bolt head, base at z=0, protruding +Z by BOLT_H."""
    geo = CylinderGeometry(BOLT_R, BOLT_H, radial_segments=6)
    geo.translate(0.0, 0.0, BOLT_H / 2.0)
    return geo


def _support_arm_geo() -> MeshGeometry:
    """Horizontal support arm box, centred in X/Z, extending y in [0, ARM_DEPTH]."""
    geo = BoxGeometry((ARM_W, ARM_DEPTH, ARM_H))
    geo.translate(0.0, ARM_DEPTH / 2.0, 0.0)
    return geo


# =============================================================================
# build
# =============================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_trash_can")

    # ---- materials -----------------------------------------------------------
    black = model.material("can_black", rgba=(0.10, 0.10, 0.11, 1.0))
    silver = model.material("lid_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    silver_dark = model.material("flap_silver", rgba=(0.66, 0.68, 0.71, 1.0))
    steel = model.material("bracket_steel", rgba=(0.52, 0.54, 0.56, 1.0))
    bolt_dark = model.material("bolt_steel", rgba=(0.42, 0.43, 0.45, 1.0))

    # ---- wall_plate  (root) --------------------------------------------------
    wall_plate = model.part("wall_plate")

    # flat plate
    plate_geo = BoxGeometry((PLATE_W, PLATE_T, PLATE_H))
    plate_geo.translate(0.0, PLATE_T / 2.0, PLATE_Z_CTR)
    wall_plate.visual(
        mesh_from_geometry(plate_geo, "plate_shell"),
        material=steel,
        name="plate_shell",
    )

    # hoop ring  (single torus, horizontal in XY plane, axis Z)
    hoop_geo = TorusGeometry(HOOP_R, HOOP_TUBE,
                             radial_segments=16, tubular_segments=48)
    hoop_geo.translate(0.0, CAN_Y, HOOP_Z)
    wall_plate.visual(
        mesh_from_geometry(hoop_geo, "hoop_ring"),
        material=steel,
        name="hoop_ring",
    )

    # support arms  (2 arms emitted via for-i loop, shared helper)
    for i in range(2):
        side = 1.0 if i == 0 else -1.0
        arm = _support_arm_geo()
        arm.translate(side * ARM_X, PLATE_T, HOOP_Z)
        wall_plate.visual(
            mesh_from_geometry(arm, f"arm_{i}"),
            material=steel,
            name=f"arm_{i}",
        )

    # mounting bolts  (4 bolts at plate corners, emitted via for-i loop)
    bolt_slots = [
        ( BOLT_X_OFF, PLATE_Z_CTR + BOLT_Z_OFF),
        (-BOLT_X_OFF, PLATE_Z_CTR + BOLT_Z_OFF),
        ( BOLT_X_OFF, PLATE_Z_CTR - BOLT_Z_OFF),
        (-BOLT_X_OFF, PLATE_Z_CTR - BOLT_Z_OFF),
    ]
    for i in range(len(bolt_slots)):
        bx, bz = bolt_slots[i]
        bolt = _bolt_head_geo()
        bolt.translate(bx, PLATE_T, bz)
        wall_plate.visual(
            mesh_from_geometry(bolt, f"bolt_{i}"),
            material=bolt_dark,
            name=f"bolt_{i}",
        )

    wall_plate.inertial = Inertial.from_geometry(
        Box((PLATE_W, CAN_Y * 2, PLATE_H)),
        mass=2.5,
        origin=Origin(xyz=(0.0, CAN_Y * 0.5, PLATE_Z_CTR)),
    )

    # ---- body  (FIXED child of wall_plate) -----------------------------------
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell(), "body_shell"),
        material=black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_TOP, length=BODY_H),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.45)),
    )
    model.articulation(
        "plate_to_body",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=body,
        origin=Origin(xyz=(0.0, CAN_Y, MOUNT_Z)),
    )

    # ---- lid  (FIXED child of body) ------------------------------------------
    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_mesh(), "lid_dome"),
        material=silver,
        name="lid_dome",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_R, length=LID_RISE),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, BODY_H + LID_RISE * 0.4)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.FIXED,
        parent=body,
        child=lid,
        origin=Origin(),
    )

    # ---- flap  (REVOLUTE child of lid – primary articulation) ----------------
    flap = model.part("flap")
    flap.visual(
        mesh_from_geometry(_flap_mesh(), "flap_cap"),
        material=silver_dark,
        name="flap_cap",
    )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_LEN, FLAP_WID, FLAP_RISE)),
        mass=0.04,
    )

    def _dome_z(r):
        t = min(1.0, max(0.0, r / LID_R))
        return BODY_H + LID_RISE * math.cos(t * math.pi / 2.0)

    pivot_z = _dome_z(FLAP_WID / 2.0) - 0.001

    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=3.0, lower=-1.6, upper=1.6
        ),
    )

    return model


# =============================================================================
# tests
# =============================================================================

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("lid_to_flap")

    # ---- wall plate shell is thin and tall (vertical steel plate) ------------
    plate_aabb = ctx.part_element_world_aabb(wall_plate, elem="plate_shell")
    pext = _ext(plate_aabb) if plate_aabb else None
    ctx.check(
        "wall plate shell is thin in Y relative to width and height",
        pext is not None and pext[1] < pext[0] * 0.5 and pext[1] < pext[2] * 0.3,
        details=f"plate_shell_ext={pext}",
    )
    ctx.check(
        "wall plate shell height >= 0.35 m",
        pext is not None and pext[2] >= 0.35,
        details=f"plate_shell_height={None if pext is None else pext[2]}",
    )

    # ---- hoop ring present and overlaps body XY footprint --------------------
    ctx.expect_overlap(
        body, wall_plate,
        axes="xy",
        elem_a="body_shell",
        elem_b="hoop_ring",
        min_overlap=0.10,
        name="hoop ring overlaps body footprint in XY (encircles the can)",
    )

    # ---- body is elevated off ground (wall-mounted, no legs) ----------------
    baabb = ctx.part_world_aabb(body)
    bext = _ext(baabb)
    ctx.check(
        "body bottom is elevated above ground (wall-mounted)",
        baabb[0][2] > 0.30,
        details=f"body_min_z={baabb[0][2]}",
    )
    ctx.check(
        "body height matches ~0.25 m",
        abs(bext[2] - BODY_H) < 0.01,
        details=f"body_ext={bext}",
    )

    # ---- body stands off the wall (forward in +Y) ---------------------------
    body_y_ctr = (baabb[0][1] + baabb[1][1]) / 2.0
    ctx.check(
        "body centre is forward from wall (y > 0.10 m)",
        body_y_ctr > 0.10,
        details=f"body_y_ctr={body_y_ctr}",
    )

    # ---- mounting bolts present on plate ------------------------------------
    bolt_names = sorted(
        v.name for v in wall_plate.visuals if v.name and v.name.startswith("bolt_")
    )
    ctx.check(
        "at least 4 mounting bolts on wall plate",
        len(bolt_names) >= 4,
        details=f"bolts={bolt_names}",
    )

    # ---- support arms present -----------------------------------------------
    arm_names = sorted(
        v.name for v in wall_plate.visuals if v.name and v.name.startswith("arm_")
    )
    ctx.check(
        "at least 2 support arms connecting plate to hoop",
        len(arm_names) >= 2,
        details=f"arms={arm_names}",
    )

    # ---- hoop ring grips body (small intentional overlap at contact) ---------
    ctx.allow_overlap(
        wall_plate, body,
        elem_a="hoop_ring",
        elem_b="body_shell",
        reason="The hoop ring slightly compresses against the body outer wall to grip and cradle the can, as a real wall-mount bracket holds the bin.",
    )
    ctx.expect_contact(
        wall_plate, body,
        elem_a="hoop_ring",
        elem_b="body_shell",
        name="hoop ring contacts body outer wall (cradle fit)",
    )

    # ---- lid seats over body mouth ------------------------------------------
    ctx.allow_overlap(
        body, lid,
        reason="The silver dome lid skirt sleeves down over the body rim, as a real swing-lid cap seats on the can.",
    )
    ctx.expect_overlap(
        lid, body, axes="xy",
        min_overlap=0.18,
        name="dome lid seats over the body mouth footprint",
    )

    # ---- swing flap seated in dome opening ----------------------------------
    ctx.allow_overlap(
        flap, lid,
        reason="The teardrop flap sits in the dome opening and overlaps the recess lip at its pivot seam.",
    )
    ctx.expect_contact(flap, lid, name="swing flap seated in the dome opening")

    # ---- primary articulation: rocker swing about Y -------------------------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ctx.check(
        "flap swing axis is lateral (local Y)",
        abs(flap_joint.axis[1]) > 0.99
        and abs(flap_joint.axis[0]) < 0.01
        and abs(flap_joint.axis[2]) < 0.01,
        details=f"axis={flap_joint.axis}",
    )
    lim = flap_joint.motion_limits
    ctx.check(
        "flap swing travel >= ~90 deg each way",
        lim is not None and lim.upper >= 1.4 and lim.lower <= -1.4,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # rocker pose: tipping shrinks footprint and drops leading edge
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.3}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap rocks: X footprint shrinks when swung",
        (open_aabb[1][0] - open_aabb[0][0]) < (rest_aabb[1][0] - rest_aabb[0][0]) - 0.02,
        details=f"rest_x={rest_aabb[1][0] - rest_aabb[0][0]}, "
                f"open_x={open_aabb[1][0] - open_aabb[0][0]}",
    )
    ctx.check(
        "swinging flap lowers leading edge below closed crown",
        open_aabb[0][2] < rest_aabb[0][2] - 0.02,
        details=f"rest_min_z={rest_aabb[0][2]}, open_min_z={open_aabb[0][2]}",
    )

    return ctx.report()


object_model = build_object_model()
