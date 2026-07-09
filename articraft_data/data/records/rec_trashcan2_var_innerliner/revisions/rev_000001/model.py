from __future__ import annotations

# Small cylindrical black plastic swing-lid trash can with a silver domed lid.
#
# Real object: a small kitchen / desk swing-top bin. A black plastic body that
# is a gently tapered cylinder (a little narrower at the floor, wider at the
# mouth), open inside. A brushed-silver domed lid caps the mouth and overhangs
# the rim. Set into the center of the dome is a teardrop SWING FLAP: a small
# curved rocker panel that tips inward when pushed and rocks back closed. The
# swing flap is the primary user-facing articulation.
#
# Coordinate convention:
#   - +Z is up; the body footprint sits at z = 0.
#   - The dome axis is +Z. The teardrop flap's long axis runs along X; its rocker
#     pivot is a horizontal Y axis through the flap center.
#
# Root structure: the body is the root; the silver dome lid is fixed to the body
# rim; the teardrop flap rocks on the dome (REVOLUTE, swing/rocker, axis Y).
#
# Articulation:
#   - lid_to_flap : REVOLUTE, swing/rocker flap, axis along Y through flap center.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
R_BOTTOM = 0.105            # body radius at the floor
R_TOP = 0.122              # body radius at the mouth (slight outward taper)
BODY_H = 0.250             # body height (mouth rim z)
WALL_T = 0.004
FLOOR_T = 0.008

LID_OVERHANG = 0.010      # how far the dome lip overhangs the rim
LID_R = R_TOP + LID_OVERHANG
LID_RISE = 0.060          # height of the dome crown above the rim
LID_SKIRT = 0.022         # vertical skirt that sleeves down over the body rim

FLAP_LEN = 0.130          # teardrop flap length (X)
FLAP_WID = 0.095          # teardrop flap width (Y)
FLAP_RISE = 0.022         # flap crown height (curved cap)

LID_TOP_Z = BODY_H + LID_RISE

# ---- inner liner dimensions -------------------------------------------------
LINER_GAP = 0.005         # radial clearance between liner OD and body ID
LINER_WALL = 0.003        # liner wall thickness
LINER_FLOOR = 0.006       # liner floor thickness
LINER_R_BOTTOM = R_BOTTOM - WALL_T - LINER_GAP  # ~0.096
LINER_R_TOP = R_TOP - WALL_T - LINER_GAP        # ~0.113
LINER_H = BODY_H - 0.020  # liner stops just below the body rim
LINER_LIFT = 0.32         # prismatic travel: enough to fully clear dome
LINER_RIM_H = 0.010       # outward-rolled rim lip height
LINER_RIM_T = 0.003       # rim lip radial thickness (must fit inside body)
NUM_GRIPS = 4             # carry grip tabs around the liner rim


def _body_shell() -> MeshGeometry:
    """Hollow tapered cylindrical body, open top, floor on z=0.

    Built as a revolved thin-wall shell so the bin is genuinely hollow.
    Profile uses (radius, z) outer/inner pairs.
    """
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
    # close the floor with a thin disc just above z=0 inside the wall
    floor = CylinderGeometry(R_BOTTOM - WALL_T, FLOOR_T, radial_segments=64)
    floor.translate(0.0, 0.0, FLOOR_T / 2.0)
    shell.merge(floor)
    return shell


def _hole_r(theta: float) -> float:
    """Teardrop boundary radius at angle theta (long axis along X)."""
    a = FLAP_LEN / 2.0
    b = FLAP_WID / 2.0
    er = (a * b) / math.hypot(b * math.cos(theta), a * math.sin(theta))
    taper = 1.0 - 0.12 * max(0.0, -math.cos(theta))  # slight point toward -X
    return er * taper


def _lid_mesh() -> MeshGeometry:
    """Silver domed lid capping the mouth, with a teardrop opening on top.

    A revolved dome shell (rounded crown) blended into a teardrop hole at the
    center, plus a short vertical skirt that sleeves over the body rim. The
    crown surface is built from the outer rim inward to the teardrop boundary
    so the flap sits in a real recess.
    """
    geo = MeshGeometry()
    rim_z = BODY_H
    cr = LID_R
    seg = 64
    rings = 10

    def dome_z(r):
        t = min(1.0, max(0.0, r / cr))
        return rim_z + LID_RISE * math.cos(t * math.pi / 2.0)

    grid_out = []
    grid_in = []
    for i in range(rings + 1):
        f = i / rings  # 0 at rim, 1 at hole boundary
        ring_out = []
        ring_in = []
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
    # connect outer & inner along the hole boundary (innermost ring) to make a lip
    last = rings
    for j in range(seg):
        jn = (j + 1) % seg
        a, b = out_idx[last][j], out_idx[last][jn]
        c, d = in_idx[last][jn], in_idx[last][j]
        geo.add_face(a, b, c); geo.add_face(a, c, d)

    # vertical skirt: a short open-ended sleeve hanging from the rim down over
    # the body. closed=False so it has NO end-cap discs -- end caps would seal
    # the can mouth right under the dome's teardrop opening.
    skirt = CylinderGeometry(cr, LID_SKIRT, radial_segments=seg, closed=False)
    skirt.translate(0.0, 0.0, rim_z - LID_SKIRT / 2.0)
    geo.merge(skirt)

    # Annular seating ring at the body rim height: a thin washer from the body
    # outer radius to the dome rim, representing the lid's flat seating surface
    # that rests on the can rim. The center stays open (no cap) so the trash
    # mouth remains a genuine void.
    seat_t = 0.003  # ring thickness
    seat_profiles_outer = [
        (R_TOP, rim_z),
        (cr, rim_z),
        (cr, rim_z - seat_t),
        (R_TOP, rim_z - seat_t),
    ]
    seat_profiles_inner = [
        (R_TOP + 0.001, rim_z - 0.001),
        (cr - 0.001, rim_z - 0.001),
        (cr - 0.001, rim_z - seat_t + 0.001),
        (R_TOP + 0.001, rim_z - seat_t + 0.001),
    ]
    seat_ring = LatheGeometry.from_shell_profiles(
        seat_profiles_outer, seat_profiles_inner,
        segments=seg, start_cap="flat", end_cap="flat"
    )
    geo.merge(seat_ring)

    return geo


def _flap_mesh() -> MeshGeometry:
    """Teardrop curved swing flap, authored centered on its own pivot.

    A flattened domed cap (curved like the lid) shaped as a teardrop. Pivot is a
    horizontal Y axis through the flap center at the local origin, so the rocker
    swing reads correctly. Local frame: long axis X, thin/curved along Z.
    """
    geo = MeshGeometry()
    seg = 48
    rings = 6

    # domed teardrop top surface, curving up toward the center
    grid = []
    for i in range(rings + 1):
        f = i / rings  # 0 center, 1 edge
        ring = []
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

    # flat underside skin at z=0
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

    # edge wall closing top to base at the rim
    for j in range(seg):
        jn = (j + 1) % seg
        t, tn = idx[rings][j], idx[rings][jn]
        bse, bsen = base_idx[rings][j], base_idx[rings][jn]
        geo.add_face(t, tn, bsen); geo.add_face(t, bsen, bse)
    return geo


def _liner_shell() -> MeshGeometry:
    """Hollow tapered inner liner bin, open top, floor just above z=0.

    A smaller version of the body shell, designed to nest inside with clearance.
    The liner floor sits at z = LINER_FLOOR_STANDOFF above the body floor.
    """
    outer = [
        (LINER_R_BOTTOM, 0.0),
        (LINER_R_BOTTOM + 0.003, 0.008),
        (LINER_R_TOP, LINER_H),
    ]
    inner = [
        (LINER_R_BOTTOM - LINER_WALL, LINER_FLOOR),
        (LINER_R_BOTTOM - LINER_WALL + 0.003, LINER_FLOOR + 0.008),
        (LINER_R_TOP - LINER_WALL, LINER_H),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer, inner, segments=64, start_cap="flat", end_cap="flat"
    )
    # close the floor
    floor = CylinderGeometry(LINER_R_BOTTOM - LINER_WALL, LINER_FLOOR, radial_segments=64)
    floor.translate(0.0, 0.0, LINER_FLOOR / 2.0)
    shell.merge(floor)
    return shell


def _liner_rim_mesh() -> MeshGeometry:
    """Outward-rolled rim lip at the top of the liner.

    A short ring that flares outward from the liner top edge, providing a
    visible lip / carry rim.
    """
    outer = [
        (LINER_R_TOP, LINER_H),
        (LINER_R_TOP + LINER_RIM_T, LINER_H),
        (LINER_R_TOP + LINER_RIM_T, LINER_H - LINER_RIM_H),
        (LINER_R_TOP, LINER_H - LINER_RIM_H),
    ]
    inner = [
        (LINER_R_TOP - LINER_WALL, LINER_H),
        (LINER_R_TOP + LINER_RIM_T - LINER_WALL, LINER_H),
        (LINER_R_TOP + LINER_RIM_T - LINER_WALL, LINER_H - LINER_RIM_H),
        (LINER_R_TOP - LINER_WALL, LINER_H - LINER_RIM_H),
    ]
    return LatheGeometry.from_shell_profiles(
        outer, inner, segments=64, start_cap="flat", end_cap="flat"
    )


def _grip_tab_mesh() -> MeshGeometry:
    """Small vertical grip tab that sits on the liner rim, extending upward.

    A thin rectangular pad mounted vertically on the rim lip, serving as a
    finger grip for lifting the liner. Centered at origin with the tab
    extending along +Z (upward) and thin along X (radial direction).
    """
    tab_radial = 0.004   # X: thin radial thickness
    tab_width = 0.022    # Y: width along rim
    tab_height = 0.016   # Z: height above rim
    geo = MeshGeometry()
    hx = tab_radial / 2
    hy = tab_width / 2
    # Box from z=0 to z=tab_height, centered on X and Y
    v = [
        geo.add_vertex(-hx, -hy, 0.0),          # 0
        geo.add_vertex( hx, -hy, 0.0),          # 1
        geo.add_vertex( hx,  hy, 0.0),          # 2
        geo.add_vertex(-hx,  hy, 0.0),          # 3
        geo.add_vertex(-hx, -hy, tab_height),    # 4
        geo.add_vertex( hx, -hy, tab_height),    # 5
        geo.add_vertex( hx,  hy, tab_height),    # 6
        geo.add_vertex(-hx,  hy, tab_height),    # 7
    ]
    # bottom
    geo.add_face(v[0], v[2], v[1]); geo.add_face(v[0], v[3], v[2])
    # top
    geo.add_face(v[4], v[5], v[6]); geo.add_face(v[4], v[6], v[7])
    # front (+X)
    geo.add_face(v[1], v[2], v[6]); geo.add_face(v[1], v[6], v[5])
    # back (-X)
    geo.add_face(v[0], v[4], v[7]); geo.add_face(v[0], v[7], v[3])
    # right (+Y)
    geo.add_face(v[2], v[3], v[7]); geo.add_face(v[2], v[7], v[6])
    # left (-Y)
    geo.add_face(v[0], v[1], v[5]); geo.add_face(v[0], v[5], v[4])
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_lid_trash_can")

    black = model.material("can_black", rgba=(0.10, 0.10, 0.11, 1.0))
    silver = model.material("lid_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    silver_dark = model.material("flap_silver", rgba=(0.66, 0.68, 0.71, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.28, 0.28, 0.30, 1.0))
    grip_black = model.material("grip_black", rgba=(0.15, 0.15, 0.16, 1.0))

    # ---- body (root) --------------------------------------------------------
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

    # ---- silver domed lid (fixed to body) -----------------------------------
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

    # ---- teardrop swing flap (rocker) on the dome ---------------------------
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
    # pivot: seat the flap down in the recess so its rim edge rests on the lid
    # lip ring. The lip (hole boundary) sits at dome_z(FLAP edge); place the
    # pivot there (flap base edge is at local z=0) with a hair of overlap so it
    # reads as truly seated, not floating.
    def _dome_z(r):
        t = min(1.0, max(0.0, r / LID_R))
        return BODY_H + LID_RISE * math.cos(t * math.pi / 2.0)
    pivot_z = _dome_z(FLAP_WID / 2.0) - 0.001
    # REVOLUTE rocker swing about the lateral (Y) axis through the flap center.
    # Positive q tips the front (+X) edge down/in (a real swing lid). Limit
    # ~ +/- 95 deg so the lid swings well past 90 deg of travel.
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

    # ---- inner liner bin (removable, lifts out vertically) ------------------
    # The liner sits nested inside the body, centered on the body axis.
    # Its floor sits a small standoff above the body floor so it doesn't
    # scrape. PRISMATIC joint along +Z lets it lift straight up and out.
    liner_standoff = 0.012  # liner floor above body floor
    liner = model.part("liner")
    liner.visual(
        mesh_from_geometry(_liner_shell(), "liner_shell"),
        material=liner_gray,
        name="liner_shell",
    )
    liner.visual(
        mesh_from_geometry(_liner_rim_mesh(), "liner_rim"),
        material=liner_gray,
        name="liner_rim",
    )
    liner.inertial = Inertial.from_geometry(
        Cylinder(radius=LINER_R_TOP, length=LINER_H),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, LINER_H * 0.45)),
    )

    # Grip tabs around the liner rim, emitted via a for-i-in-range loop
    # with a shared geometry helper and name_i naming.
    grip_geo = _grip_tab_mesh()
    for i in range(NUM_GRIPS):
        theta = 2.0 * math.pi * i / NUM_GRIPS
        # Place each tab on top of the rim lip, centered radially on the rim
        r_mount = LINER_R_TOP + LINER_RIM_T / 2.0  # center of rim lip
        gx = r_mount * math.cos(theta)
        gy = r_mount * math.sin(theta)
        gz = LINER_H  # sits on top of the liner wall
        tab_name = f"grip_{i}"
        liner.visual(
            mesh_from_geometry(grip_geo, tab_name),
            material=grip_black,
            name=tab_name,
            origin=Origin(
                xyz=(gx, gy, gz),
                rpy=(0.0, 0.0, theta),
            ),
        )

    # PRISMATIC joint: liner lifts straight up (+Z) out of the body.
    # At q=0 the liner is seated; at q=LINER_LIFT it fully clears the dome.
    model.articulation(
        "body_to_liner",
        ArticulationType.PRISMATIC,
        parent=body,
        child=liner,
        origin=Origin(xyz=(0.0, 0.0, liner_standoff)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=LINER_LIFT
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    flap = object_model.get_part("flap")
    liner = object_model.get_part("liner")
    flap_joint = object_model.get_articulation("lid_to_flap")
    liner_joint = object_model.get_articulation("body_to_liner")

    # --- base sits at z ~ 0; body height & diameter realistic ----------------
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "body base sits at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.004,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    bext = _ext(baabb)
    ctx.check(
        "body height matches ~0.25 m",
        abs(bext[2] - BODY_H) < 0.01,
        details=f"body_ext={bext}",
    )
    ctx.check(
        "body is a ~0.24 m diameter cylinder",
        0.20 < bext[0] < 0.27 and 0.20 < bext[1] < 0.27,
        details=f"body_ext={bext}",
    )

    # --- silver dome lid caps the body, overhangs the rim --------------------
    laabb = ctx.part_world_aabb(lid)
    lext = _ext(laabb)
    ctx.check(
        "dome lid crown rises above the body rim",
        laabb[1][2] > baabb[1][2] + LID_RISE * 0.6,
        details=f"lid_top={laabb[1][2]}, body_top={baabb[1][2]}",
    )
    ctx.check(
        "dome lid overhangs the body rim",
        lext[0] > bext[0] + 0.005 and lext[1] > bext[1] + 0.005,
        details=f"lid_ext={lext}, body_ext={bext}",
    )
    # lid skirt sleeves down over the body rim (seated cap, intentional overlap)
    ctx.allow_overlap(
        body, lid,
        reason="The silver dome lid skirt sleeves down over the body rim, as a real swing-lid cap seats on the can.",
    )
    ctx.expect_overlap(
        lid, body, axes="xy",
        min_overlap=0.18,
        name="dome lid seats over the body mouth footprint",
    )

    # --- teardrop swing flap sized & seated in the dome opening --------------
    faabb = ctx.part_world_aabb(flap)
    fext = _ext(faabb)
    ctx.check(
        "swing flap is a teardrop ~0.13 x 0.095 m",
        abs(fext[0] - FLAP_LEN) < 0.02 and abs(fext[1] - FLAP_WID) < 0.02,
        details=f"flap_ext={fext}",
    )
    ctx.check(
        "swing flap is centered on the dome crown",
        abs((faabb[0][0] + faabb[1][0]) / 2.0) < 0.02
        and abs((faabb[0][1] + faabb[1][1]) / 2.0) < 0.02,
        details=f"flap_center=({(faabb[0][0]+faabb[1][0])/2.0},{(faabb[0][1]+faabb[1][1])/2.0})",
    )
    ctx.allow_overlap(
        flap, lid,
        reason="The teardrop flap sits in the dome opening and overlaps the recess lip at its pivot seam.",
    )
    ctx.expect_contact(flap, lid, name="swing flap seated in the dome opening")

    # --- primary articulation: rocker swing about Y --------------------------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ctx.check(
        "flap swing axis is lateral (local Y)",
        abs(flap_joint.axis[1]) > 0.99 and abs(flap_joint.axis[0]) < 0.01
        and abs(flap_joint.axis[2]) < 0.01,
        details=f"axis={flap_joint.axis}",
    )
    lim = flap_joint.motion_limits
    ctx.check(
        "flap swing travel is realistic (>= ~90 deg each way)",
        lim is not None and lim.lower is not None and lim.upper is not None
        and lim.upper >= 1.4 and lim.lower <= -1.4,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # rocker behavior: tipping one way drops the +X edge and lifts the -X edge.
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.3}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap rocks about its center: footprint shrinks in X",
        (open_aabb[1][0] - open_aabb[0][0]) < (rest_aabb[1][0] - rest_aabb[0][0]) - 0.02,
        details=f"rest_x_ext={rest_aabb[1][0]-rest_aabb[0][0]}, open_x_ext={open_aabb[1][0]-open_aabb[0][0]}",
    )
    ctx.check(
        "swinging the flap lowers its leading edge below the closed crown (opens)",
        open_aabb[0][2] < rest_aabb[0][2] - 0.02,
        details=f"rest_min_z={rest_aabb[0][2]}, open_min_z={open_aabb[0][2]}",
    )

    # --- inner liner: nested bin with prismatic lift-out ----------------------
    laabb_liner = ctx.part_world_aabb(liner)
    lext_liner = _ext(laabb_liner)

    ctx.check(
        "liner is a cylinder nested inside the body",
        lext_liner[0] < bext[0] - 0.008 and lext_liner[1] < bext[1] - 0.008,
        details=f"liner_ext={lext_liner}, body_ext={bext}",
    )
    ctx.check(
        "liner height is close to body height",
        abs(lext_liner[2] - LINER_H) < 0.02,
        details=f"liner_h={lext_liner[2]}, expected={LINER_H}",
    )

    # liner XY footprint is within the body XY footprint (nested)
    ctx.expect_within(
        liner, body, axes="xy",
        margin=0.0,
        name="liner XY footprint stays within body",
    )

    # liner floor is above body floor (standoff)
    ctx.check(
        "liner floor sits above body floor",
        laabb_liner[0][2] > baabb[0][2] + 0.004,
        details=f"liner_min_z={laabb_liner[0][2]}, body_min_z={baabb[0][2]}",
    )

    # liner shell top is below body top rim (grip tabs may protrude for grab)
    ctx.check(
        "liner shell top stays below body rim when seated",
        laabb_liner[1][2] < baabb[1][2] + 0.020,
        details=f"liner_top={laabb_liner[1][2]}, body_top={baabb[1][2]}",
    )

    # prismatic joint checks
    ctx.check(
        "liner joint is prismatic",
        str(liner_joint.articulation_type).endswith("PRISMATIC"),
        details=f"type={liner_joint.articulation_type}",
    )
    ctx.check(
        "liner lift axis is vertical (+Z)",
        abs(liner_joint.axis[2]) > 0.99 and abs(liner_joint.axis[0]) < 0.01
        and abs(liner_joint.axis[1]) < 0.01,
        details=f"axis={liner_joint.axis}",
    )
    lim_l = liner_joint.motion_limits
    ctx.check(
        "liner lift travel has realistic range",
        lim_l is not None and lim_l.lower is not None and lim_l.upper is not None
        and lim_l.upper >= 0.25 and lim_l.lower <= 0.01,
        details=f"lower={lim_l.lower}, upper={lim_l.upper}",
    )

    # prove the liner actually lifts upward when the joint is actuated
    rest_pos = ctx.part_world_position(liner)
    with ctx.pose({liner_joint: LINER_LIFT}):
        lifted_pos = ctx.part_world_position(liner)
    ctx.check(
        "liner lifts upward when prismatic joint is extended",
        rest_pos is not None and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.20,
        details=f"rest_z={rest_pos}, lifted_z={lifted_pos}",
    )

    # at max lift, the liner bottom clears the body top (fully removed)
    with ctx.pose({liner_joint: LINER_LIFT}):
        lifted_aabb = ctx.part_world_aabb(liner)
    ctx.check(
        "fully lifted liner clears above the body rim",
        lifted_aabb[0][2] > baabb[1][2] - 0.02,
        details=f"lifted_bottom={lifted_aabb[0][2]}, body_top={baabb[1][2]}",
    )

    # Grip tabs: verify at least 4 grip visuals exist on the liner
    liner_visuals = [v for v in liner.visuals]
    grip_names = [v.name for v in liner_visuals if v.name and v.name.startswith("grip_")]
    ctx.check(
        f"liner has {NUM_GRIPS} grip tabs",
        len(grip_names) >= NUM_GRIPS,
        details=f"found grips={grip_names}",
    )

    return ctx.report()


object_model = build_object_model()
