from __future__ import annotations

# Street/public trash can with a domed push-flap top.
#
# Real object: a cylindrical public street bin, black plastic body, gently
# tapered (narrower at the floor, wider at the mouth). A brushed-silver domed
# lid caps the mouth and overhangs the rim. Centered in the dome crown is a
# small CIRCULAR push flap: a compact round spring-style panel that pivots
# inward when pushed and rocks back closed (REVOLUTE rocker about a horizontal
# Y axis through the flap center).
#
# CRITICAL: the dome has a real circular opening under the push flap. Pushing
# the flap reveals the hollow bin interior. The dome mesh is built as a shell
# grid that goes from the outer rim inward to the circular hole boundary, with
# no cap face across the hole.
#
# Coordinate convention:
#   +Z is up; body footprint sits at z = 0.
#   The dome axis is +Z. The flap pivot is a horizontal Y axis through center.
#
# Root structure: body is root; dome lid fixed to body rim; push flap rocks on
# the dome (REVOLUTE, swing/rocker, axis Y).

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
# Street/public trash can proportions
R_BOTTOM = 0.155            # body radius at the floor
R_TOP = 0.180               # body radius at the mouth (slight outward taper)
BODY_H = 0.850              # body height (mouth rim z)
WALL_T = 0.005
FLOOR_T = 0.008

LID_OVERHANG = 0.012        # dome lip overhang beyond rim
LID_R = R_TOP + LID_OVERHANG
LID_RISE = 0.090            # dome crown height above the rim
LID_SKIRT = 0.028           # vertical skirt that sleeves over the body rim

# Circular push flap (compact round panel in the dome crown)
FLAP_RADIUS = 0.056         # push flap radius
FLAP_GAP = -0.001           # negative: flap edge embeds 1mm into dome lip shell
HOLE_RADIUS = FLAP_RADIUS + FLAP_GAP  # dome opening radius (0.055)
FLAP_RISE = 0.010           # slight dome crown on the flap itself
FLAP_THICK = 0.004          # flap panel thickness

LID_TOP_Z = BODY_H + LID_RISE


def _body_shell() -> MeshGeometry:
    """Hollow tapered cylindrical body, open top, floor on z=0.

    Built as a revolved thin-wall shell so the bin is genuinely hollow.
    """
    outer = [
        (R_BOTTOM, 0.0),
        (R_BOTTOM + 0.005, 0.012),
        (R_TOP, BODY_H),
    ]
    inner = [
        (R_BOTTOM - WALL_T, FLOOR_T),
        (R_BOTTOM - WALL_T + 0.005, FLOOR_T + 0.012),
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


def _dome_z(r: float) -> float:
    """Dome surface height at radius r from center axis."""
    t = min(1.0, max(0.0, r / LID_R))
    return BODY_H + LID_RISE * math.cos(t * math.pi / 2.0)


def _lid_mesh() -> MeshGeometry:
    """Silver domed lid with a CIRCULAR opening at the crown.

    Built as a shell grid from the outer rim inward to the circular hole
    boundary. No cap face spans the hole, so pushing the flap reveals the
    hollow bin interior. A vertical skirt sleeves down over the body rim.
    """
    geo = MeshGeometry()
    rim_z = BODY_H
    cr = LID_R
    hr = HOLE_RADIUS
    seg = 64
    rings = 12

    # Grid from outer rim (ring 0) to hole boundary (ring `rings`)
    grid_out = []
    grid_in = []
    for i in range(rings + 1):
        f = i / rings  # 0 at rim, 1 at hole boundary
        ring_out = []
        ring_in = []
        for j in range(seg):
            th = 2.0 * math.pi * j / seg
            r = cr + (hr - cr) * f
            z = _dome_z(r)
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

    # Connect outer & inner along the hole boundary (innermost ring) for lip
    last = rings
    for j in range(seg):
        jn = (j + 1) % seg
        a, b = out_idx[last][j], out_idx[last][jn]
        c, d = in_idx[last][jn], in_idx[last][j]
        geo.add_face(a, b, c); geo.add_face(a, c, d)

    # Annular rim ring: flat plate bridging from body outer wall to dome outer
    # edge at the rim level. This ensures the lid physically contacts the body.
    rim_inner = R_TOP - 0.001  # slight embed into body outer wall for contact
    for j in range(seg):
        jn = (j + 1) % seg
        th0 = 2.0 * math.pi * j / seg
        th1 = 2.0 * math.pi * jn / seg
        # top face at rim_z
        ri0 = geo.add_vertex(rim_inner * math.cos(th0), rim_inner * math.sin(th0), rim_z)
        ri1 = geo.add_vertex(rim_inner * math.cos(th1), rim_inner * math.sin(th1), rim_z)
        ro0 = out_idx[0][j]
        ro1 = out_idx[0][jn]
        geo.add_face(ri0, ro0, ro1); geo.add_face(ri0, ro1, ri1)
        # bottom face (underside of rim ring)
        rib0 = geo.add_vertex(rim_inner * math.cos(th0), rim_inner * math.sin(th0), rim_z - 0.003)
        rib1 = geo.add_vertex(rim_inner * math.cos(th1), rim_inner * math.sin(th1), rim_z - 0.003)
        # skirt top edge vertices
        sko = geo.add_vertex(cr * math.cos(th0), cr * math.sin(th0), rim_z - 0.003)
        skn = geo.add_vertex(cr * math.cos(th1), cr * math.sin(th1), rim_z - 0.003)
        geo.add_face(rib0, rib1, skn); geo.add_face(rib0, skn, sko)

    # Vertical skirt: open-ended sleeve hanging from rim down over the body
    skirt = CylinderGeometry(cr, LID_SKIRT, radial_segments=seg, closed=False)
    skirt.translate(0.0, 0.0, rim_z - LID_SKIRT / 2.0)
    geo.merge(skirt)
    return geo


def _flap_mesh() -> MeshGeometry:
    """Circular push flap: a slightly domed round panel.

    Authored centered on its own pivot at the local origin. The flap is a thin
    disc with a slight dome crown, with its flat underside at z=0 and the crown
    rising to FLAP_RISE above. Pivot is horizontal Y axis through center.
    """
    geo = MeshGeometry()
    seg = 48
    rings = 6
    fr = FLAP_RADIUS

    # Domed top surface
    grid = []
    for i in range(rings + 1):
        f = i / rings  # 0 center, 1 edge
        ring = []
        for j in range(seg):
            th = 2.0 * math.pi * j / seg
            r = fr * f
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

    # Flat underside at z=0
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

    # Edge wall closing top to base at rim
    for j in range(seg):
        jn = (j + 1) % seg
        t, tn = idx[rings][j], idx[rings][jn]
        bse, bsen = base_idx[rings][j], base_idx[rings][jn]
        geo.add_face(t, tn, bsen); geo.add_face(t, bsen, bse)

    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_flap_trash_can")

    black = model.material("can_black", rgba=(0.10, 0.10, 0.11, 1.0))
    silver = model.material("lid_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    silver_dark = model.material("flap_silver", rgba=(0.62, 0.64, 0.67, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell(), "body_shell"),
        material=black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_TOP, length=BODY_H),
        mass=2.5,
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
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, BODY_H + LID_RISE * 0.4)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.FIXED,
        parent=body,
        child=lid,
        origin=Origin(),
    )

    # ---- circular push flap (rocker) on the dome crown ----------------------
    flap = model.part("push_flap")
    flap.visual(
        mesh_from_geometry(_flap_mesh(), "flap_panel"),
        material=silver_dark,
        name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Cylinder(radius=FLAP_RADIUS, length=FLAP_THICK),
        mass=0.03,
    )

    # Pivot: seat the flap in the dome opening. The flap base (z=0 local) sits
    # at the dome crown height at the hole boundary, with a hair of overlap so
    # it reads as truly seated in the recess lip.
    pivot_z = _dome_z(HOLE_RADIUS) - 0.001

    # REVOLUTE rocker: horizontal Y axis through flap center.
    # Positive q tips the +X edge downward (inward into the can), revealing the
    # hollow interior through the dome opening. Travel ~±70 deg for a spring
    # push-flap.
    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=3.0, lower=-1.2, upper=1.2
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
    flap = object_model.get_part("push_flap")
    flap_joint = object_model.get_articulation("lid_to_flap")

    # --- street-can proportions: body at z=0, ~0.85m tall, ~0.36m dia -------
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "body base sits at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.005,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    bext = _ext(baabb)
    ctx.check(
        "body height is street-can scale (~0.85m)",
        abs(bext[2] - BODY_H) < 0.015,
        details=f"body_ext={bext}",
    )
    ctx.check(
        "body diameter is street-can scale (~0.30-0.40m)",
        0.28 < bext[0] < 0.42 and 0.28 < bext[1] < 0.42,
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
    # Lid skirt sleeves over body rim (intentional overlap)
    ctx.allow_overlap(
        body, lid,
        reason="The dome lid skirt sleeves down over the body rim, as a real push-flap cap seats on the can.",
    )
    ctx.expect_overlap(
        lid, body, axes="xy",
        min_overlap=0.25,
        name="dome lid seats over the body mouth footprint",
    )

    # --- circular push flap: compact round panel in dome crown ---------------
    faabb = ctx.part_world_aabb(flap)
    fext = _ext(faabb)
    flap_dia_x = fext[0]
    flap_dia_y = fext[1]
    ctx.check(
        "push flap is circular (~same X and Y extent)",
        abs(flap_dia_x - flap_dia_y) < 0.015,
        details=f"flap_x={flap_dia_x}, flap_y={flap_dia_y}",
    )
    ctx.check(
        "push flap diameter is compact (~0.10-0.12m)",
        0.08 < flap_dia_x < 0.14 and 0.08 < flap_dia_y < 0.14,
        details=f"flap_ext={fext}",
    )
    ctx.check(
        "push flap is centered on the dome crown",
        abs((faabb[0][0] + faabb[1][0]) / 2.0) < 0.02
        and abs((faabb[0][1] + faabb[1][1]) / 2.0) < 0.02,
        details=f"flap_center=({(faabb[0][0]+faabb[1][0])/2.0},{(faabb[0][1]+faabb[1][1])/2.0})",
    )

    # Flap sits in the dome opening (intentional overlap at pivot lip)
    ctx.allow_overlap(
        flap, lid,
        reason="The push flap sits in the dome circular opening and overlaps the recess lip at its pivot seam.",
    )
    ctx.expect_contact(flap, lid, name="push flap seated in the dome opening")

    # --- primary articulation: rocker push-flap about Y ----------------------
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
        "flap swing travel is realistic push-flap range (>= ~60 deg each way)",
        lim is not None and lim.lower is not None and lim.upper is not None
        and lim.upper >= 1.0 and lim.lower <= -1.0,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # Rocker behavior: positive q tips the +X edge down into the can interior.
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "pushing the flap rocks it: footprint shrinks in X",
        (open_aabb[1][0] - open_aabb[0][0]) < (rest_aabb[1][0] - rest_aabb[0][0]) - 0.01,
        details=f"rest_x_ext={rest_aabb[1][0]-rest_aabb[0][0]}, open_x_ext={open_aabb[1][0]-open_aabb[0][0]}",
    )
    ctx.check(
        "pushing the flap lowers its leading edge (opens inward)",
        open_aabb[0][2] < rest_aabb[0][2] - 0.01,
        details=f"rest_min_z={rest_aabb[0][2]}, open_min_z={open_aabb[0][2]}",
    )

    # Dome opening is a real void: the flap center at rest sits above the body
    # interior (the opening reveals hollow inside, not a solid cap).
    ctx.check(
        "flap rests at dome crown height (above body rim)",
        rest_aabb[0][2] > BODY_H - 0.01,
        details=f"flap_min_z={rest_aabb[0][2]}, body_rim_z={BODY_H}",
    )

    return ctx.report()


object_model = build_object_model()
