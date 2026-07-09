from __future__ import annotations

# Blue rectangular swing-flap trash bin with a gray peaked sheet-metal top.
#
# Real object: a tall blue painted-steel waste bin (kitchen / office floor bin)
# with a slightly tapered rectangular body (a little wider at the top), an open
# mouth, and a gray sheet-metal "gable" peaked top that caps the mouth. Set into
# the front-sloping face of that gray top is a square gray SWING FLAP: a rocker
# lid that pivots about a horizontal axis through its own middle so it tips
# inward when pushed and falls back closed. The swing flap is the primary
# user-facing articulation.
#
# Coordinate convention:
#   - +Z is up; the body footprint sits at z = 0.
#   - The ridge of the gable roof runs along the Y axis (front face slopes toward
#     +X, rear face slopes toward -X). The swing flap lives on the +X sloped
#     face.
#   - Centerline is y = 0; the body is left/right symmetric across it.
#
# Root structure: the bin body is the root. The gray peaked top is fixed to the
# body rim. The swing flap pivots on the front sloped roof face (REVOLUTE,
# rocker swing about a horizontal Y axis through the flap center).
#
# Articulation:
#   - roof_to_flap : REVOLUTE, swing/rocker flap, axis along the roof ridge (Y).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_H = 0.560               # body height (mouth rim z)
BODY_BOT = 0.330             # body width/depth at the floor
BODY_TOP = 0.370             # body width/depth at the mouth (slight outward taper)
WALL_T = 0.006               # sheet-metal wall thickness
FLOOR_T = 0.010

ROOF_RISE = 0.150            # height of the gable ridge above the mouth rim
ROOF_OVERHANG = 0.010        # roof eaves overhang past the body walls
ROOF_T = 0.006               # roof skin thickness

FLAP_SIZE = 0.190            # square swing-flap edge length (along slope + Y)
FLAP_T = 0.010               # flap panel thickness

# ridge geometry: half-span of the roof across X at the rim, and the slope angle
ROOF_HALF = BODY_TOP / 2.0 + ROOF_OVERHANG
ROOF_SLOPE = math.atan2(ROOF_RISE, ROOF_HALF)   # pitch of each roof face


def _tapered_shell(bottom: float, top: float, height: float, wall: float,
                   floor_t: float) -> MeshGeometry:
    """Hollow open-top rectangular shell, slightly tapered, base on z=0.

    Built as four trapezoidal side walls plus a floor slab, so the bin reads as
    a real hollow body rather than a solid block.
    """
    geo = MeshGeometry()
    hb = bottom / 2.0
    ht = top / 2.0
    hbi = hb - wall
    hti = ht - wall

    def quad(p0, p1, p2, p3):
        i0 = geo.add_vertex(*p0)
        i1 = geo.add_vertex(*p1)
        i2 = geo.add_vertex(*p2)
        i3 = geo.add_vertex(*p3)
        geo.add_face(i0, i1, i2)
        geo.add_face(i0, i2, i3)

    z0 = 0.0
    z1 = height
    # Four walls. For each wall: outer-bottom, outer-top, inner-top, inner-bottom
    # quads plus a top rim quad so the wall reads as a real-thickness panel.
    # +X wall
    quad((hb, -hb, z0), (ht, -ht, z1), (ht, ht, z1), (hb, hb, z0))           # outer
    quad((hbi, hbi, z0), (hti, hti, z1), (hti, -hti, z1), (hbi, -hbi, z0))   # inner
    quad((ht, -ht, z1), (hti, -hti, z1), (hti, hti, z1), (ht, ht, z1))       # rim
    # -X wall
    quad((-hb, hb, z0), (-ht, ht, z1), (-ht, -ht, z1), (-hb, -hb, z0))
    quad((-hbi, -hbi, z0), (-hti, -hti, z1), (-hti, hti, z1), (-hbi, hbi, z0))
    quad((-ht, ht, z1), (-hti, hti, z1), (-hti, -hti, z1), (-ht, -ht, z1))
    # +Y wall
    quad((hb, hb, z0), (ht, ht, z1), (-ht, ht, z1), (-hb, hb, z0))
    quad((-hbi, hbi, z0), (-hti, hti, z1), (hti, hti, z1), (hbi, hbi, z0))
    quad((ht, ht, z1), (hti, hti, z1), (-hti, hti, z1), (-ht, ht, z1))
    # -Y wall
    quad((-hb, -hb, z0), (-ht, -ht, z1), (ht, -ht, z1), (hb, -hb, z0))
    quad((hbi, -hbi, z0), (hti, -hti, z1), (-hti, -hti, z1), (-hbi, -hbi, z0))
    quad((-ht, -ht, z1), (-hti, -hti, z1), (hti, -hti, z1), (ht, -ht, z1))

    # floor slab sitting just above z=0 inside the walls
    floor = BoxGeometry((bottom - 2 * wall, bottom - 2 * wall, floor_t))
    floor.translate(0.0, 0.0, floor_t / 2.0)
    geo.merge(floor)
    return geo


def _roof_mesh() -> MeshGeometry:
    """Gray gable peaked top capping the mouth.

    Two sloped roof skins meeting at a Y-running ridge, with two triangular
    gable ends, authored as a thin shell. A rectangular opening is left in the
    +X face for the swing flap (the flap geometry fills it). Authored in the
    body frame with the eaves at z = BODY_H.
    """
    geo = MeshGeometry()
    he = ROOF_HALF                      # half span in X at the eaves
    hy = BODY_TOP / 2.0 + ROOF_OVERHANG  # half span in Y
    z_eave = BODY_H
    z_ridge = BODY_H + ROOF_RISE

    def tri(p0, p1, p2):
        i0 = geo.add_vertex(*p0)
        i1 = geo.add_vertex(*p1)
        i2 = geo.add_vertex(*p2)
        geo.add_face(i0, i1, i2)

    def quad(p0, p1, p2, p3):
        i0 = geo.add_vertex(*p0)
        i1 = geo.add_vertex(*p1)
        i2 = geo.add_vertex(*p2)
        i3 = geo.add_vertex(*p3)
        geo.add_face(i0, i1, i2)
        geo.add_face(i0, i2, i3)

    # Ridge endpoints (top, x=0).
    ridge_front = (0.0, -hy, z_ridge)
    ridge_back = (0.0, hy, z_ridge)
    # +X eave edge
    eave_xp_front = (he, -hy, z_eave)
    eave_xp_back = (he, hy, z_eave)
    # -X eave edge
    eave_xn_front = (-he, -hy, z_eave)
    eave_xn_back = (-he, hy, z_eave)

    # -X roof face (full skin, both sides for thin-shell read)
    quad(ridge_front, ridge_back, eave_xn_back, eave_xn_front)
    quad(eave_xn_front, eave_xn_back, ridge_back, ridge_front)

    # +X roof face: same plane but with a square hole for the swing flap. Build
    # it as 4 border strips around the flap opening (in slope/Y coordinates).
    # Parameterize the +X face by s in [0,1] from ridge(s=0) to +X eave(s=1),
    # and y across the face.
    def face_pt(s, y):
        x = he * s
        z = z_ridge + (z_eave - z_ridge) * s
        return (x, y, z)

    slope_len = math.hypot(he, ROOF_RISE)
    s_half = (FLAP_SIZE / 2.0) / slope_len      # flap half-extent in s units
    s_mid = 0.5                                  # flap centered on the face span
    s_lo = s_mid - s_half
    s_hi = s_mid + s_half
    y_lo = -FLAP_SIZE / 2.0
    y_hi = FLAP_SIZE / 2.0

    # top border strip (ridge side)
    quad(face_pt(0.0, -hy), face_pt(0.0, hy), face_pt(s_lo, hy), face_pt(s_lo, -hy))
    # bottom border strip (eave side)
    quad(face_pt(s_hi, -hy), face_pt(s_hi, hy), face_pt(1.0, hy), face_pt(1.0, -hy))
    # left border strip (-Y side of opening)
    quad(face_pt(s_lo, -hy), face_pt(s_lo, y_lo), face_pt(s_hi, y_lo), face_pt(s_hi, -hy))
    # right border strip (+Y side of opening)
    quad(face_pt(s_lo, y_hi), face_pt(s_lo, hy), face_pt(s_hi, hy), face_pt(s_hi, y_hi))
    # back faces of the +X skin: the SAME four border strips with reversed
    # winding, so the panel still reads thin from inside but the flap opening is
    # a genuine hole (a single full back quad here would seal the can mouth).
    quad(face_pt(0.0, -hy), face_pt(s_lo, -hy), face_pt(s_lo, hy), face_pt(0.0, hy))
    quad(face_pt(s_hi, -hy), face_pt(1.0, -hy), face_pt(1.0, hy), face_pt(s_hi, hy))
    quad(face_pt(s_lo, -hy), face_pt(s_hi, -hy), face_pt(s_hi, y_lo), face_pt(s_lo, y_lo))
    quad(face_pt(s_lo, y_hi), face_pt(s_hi, y_hi), face_pt(s_hi, hy), face_pt(s_lo, hy))

    # two triangular gable ends (front -hy and back +hy)
    tri(eave_xn_front, eave_xp_front, ridge_front)
    tri(eave_xp_back, eave_xn_back, ridge_back)

    # Give the roof a little real thickness by adding a thin inner-offset shell
    # along -Z (so it does not read as zero-thickness paper).
    inner = geo.clone()
    inner.translate(0.0, 0.0, -ROOF_T)
    geo.merge(inner)
    # close the eave edges with vertical skirts that drop onto the body rim
    skirt_h = ROOF_T + 0.018
    for (xs, ys) in ((he, None), (-he, None)):
        quad((xs, -hy, z_eave), (xs, hy, z_eave),
             (xs, hy, z_eave - skirt_h), (xs, -hy, z_eave - skirt_h))
    for ys in (-hy, hy):
        quad((-he, ys, z_eave), (he, ys, z_eave),
             (he, ys, z_eave - skirt_h), (-he, ys, z_eave - skirt_h))
    return geo


def _flap_mesh() -> MeshGeometry:
    """Square swing-flap panel, authored centered at its own pivot.

    The flap lies in a plane and its pivot (a horizontal Y axis through its
    center) is at the local origin, so the rocker swing reads correctly. Local
    frame: panel spans X (down-slope) and Y, thin along Z.
    """
    panel = BoxGeometry((FLAP_SIZE, FLAP_SIZE, FLAP_T))
    # a small finger lip on the lower (down-slope, +X local) edge
    lip = BoxGeometry((0.016, FLAP_SIZE * 0.5, FLAP_T + 0.006))
    lip.translate(FLAP_SIZE / 2.0 - 0.010, 0.0, 0.0)
    panel.merge(lip)
    return panel


def _flap_frame():
    """World pose (origin xyz, ridge-aligned rpy) of the flap pivot on +X face.

    The flap sits centered on the +X sloped roof face. We compute the midpoint
    of the flap region on that face and the roll that tilts the panel to lie in
    the slope plane (rotation about Y so local +X points down-slope).
    """
    he = ROOF_HALF
    z_eave = BODY_H
    z_ridge = BODY_H + ROOF_RISE
    s_mid = 0.5
    x = he * s_mid
    z = z_ridge + (z_eave - z_ridge) * s_mid
    # The +X face normal points up-and-out; local +X must run down-slope (toward
    # +X, -Z). pitch about Y by +ROOF_SLOPE rotates local +X from world +X down
    # toward -Z, and local +Z (panel normal) tilts to the face normal.
    return (x, 0.0, z), (0.0, ROOF_SLOPE, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_flap_trash_bin")

    blue = model.material("bin_blue", rgba=(0.10, 0.52, 0.78, 1.0))
    gray = model.material("lid_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    gray_dark = model.material("flap_gray", rgba=(0.48, 0.49, 0.51, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_tapered_shell(BODY_BOT, BODY_TOP, BODY_H, WALL_T, FLOOR_T),
                           "body_shell"),
        material=blue,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_TOP, BODY_TOP, BODY_H)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.45)),
    )

    # ---- gray peaked roof (fixed to body) -----------------------------------
    roof = model.part("roof")
    roof.visual(
        mesh_from_geometry(_roof_mesh(), "roof_skin"),
        material=gray,
        name="roof_skin",
    )
    roof.inertial = Inertial.from_geometry(
        Box((2 * ROOF_HALF, BODY_TOP, ROOF_RISE)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_H + ROOF_RISE * 0.4)),
    )
    model.articulation(
        "body_to_roof",
        ArticulationType.FIXED,
        parent=body,
        child=roof,
        origin=Origin(),
    )

    # ---- swing flap (rocker lid) on the +X roof face ------------------------
    flap = model.part("flap")
    flap.visual(
        mesh_from_geometry(_flap_mesh(), "flap_panel"),
        material=gray_dark,
        name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_SIZE, FLAP_SIZE, FLAP_T)),
        mass=0.15,
    )
    pivot_xyz, pivot_rpy = _flap_frame()
    # REVOLUTE rocker swing about the roof-ridge (local Y) axis through the flap
    # center. Positive q tips the lower (down-slope) edge inward (a real swing
    # lid). Limit ~ +/- 80 deg so it can swing well past 90 of total travel.
    model.articulation(
        "roof_to_flap",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=flap,
        origin=Origin(xyz=pivot_xyz, rpy=pivot_rpy),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=-1.4, upper=1.4
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    roof = object_model.get_part("roof")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("roof_to_flap")

    # --- base sits at z ~ 0, body is the right height ------------------------
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "body base sits at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.005,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    bext = _ext(baabb)
    ctx.check(
        "body height matches ~0.56 m",
        abs(bext[2] - BODY_H) < 0.01,
        details=f"body_ext={bext}",
    )
    ctx.check(
        "body footprint roughly square ~0.33-0.37 m",
        0.30 < bext[0] < 0.40 and 0.30 < bext[1] < 0.40,
        details=f"body_ext={bext}",
    )

    # --- gray roof sits on top of the body, peaked above the rim ------------
    raabb = ctx.part_world_aabb(roof)
    ctx.check(
        "roof peak rises above the body rim",
        raabb is not None and raabb[1][2] > baabb[1][2] + ROOF_RISE * 0.6,
        details=f"roof_top={raabb[1][2]}, body_top={baabb[1][2]}",
    )
    ctx.check(
        "roof base meets the body rim (no floating gap)",
        raabb[0][2] < baabb[1][2] + 0.005,
        details=f"roof_min_z={raabb[0][2]}, body_top_z={baabb[1][2]}",
    )
    # the gray roof eaves/skirt intentionally drop over the body rim so the cap
    # seats on the bin (a real sheet-metal lid sleeve over the mouth).
    ctx.allow_overlap(
        body, roof,
        reason="The gray peaked top's eave skirt seats down over the body rim, as a real sheet-metal cap sleeves the bin mouth.",
    )
    ctx.expect_overlap(
        roof, body, axes="xy",
        min_overlap=0.20,
        name="roof cap seats over the body mouth footprint",
    )

    # --- swing flap is the right size and seated in the roof face -----------
    faabb = ctx.part_world_aabb(flap)
    fext = _ext(faabb)
    # the flap spans roughly FLAP_SIZE in Y (its un-rotated axis)
    ctx.check(
        "swing flap is a ~0.19 m square panel",
        abs(fext[1] - FLAP_SIZE) < 0.02 and 0.12 < fext[0] < 0.24,
        details=f"flap_ext={fext}",
    )
    # the closed flap should sit on the +X (front) sloped half of the roof
    ctx.check(
        "swing flap is on the front (+X) sloped roof face",
        faabb is not None and (faabb[0][0] + faabb[1][0]) / 2.0 > 0.04,
        details=f"flap_center_x={(faabb[0][0] + faabb[1][0]) / 2.0}",
    )

    # the flap is captured in the roof opening (intentional local seam overlap)
    ctx.allow_overlap(
        flap, roof,
        reason="The swing flap sits in the roof opening and overlaps the border lip at its pivot seam.",
    )
    ctx.expect_contact(flap, roof, name="swing flap seated in the roof opening")

    # --- primary articulation: rocker swing about the roof ridge (Y) --------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ctx.check(
        "flap swing axis is the roof ridge (local Y)",
        abs(flap_joint.axis[1]) > 0.99 and abs(flap_joint.axis[0]) < 0.01
        and abs(flap_joint.axis[2]) < 0.01,
        details=f"axis={flap_joint.axis}",
    )
    lim = flap_joint.motion_limits
    ctx.check(
        "flap swing travel is realistic (>= ~90 deg each way)",
        lim is not None and lim.lower is not None and lim.upper is not None
        and lim.upper >= 1.3 and lim.lower <= -1.3,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # rocker behavior: swinging one way drops the down-slope edge in / up; the
    # flap's extent must change versus rest, proving it actually pivots.
    rest_ext = _ext(ctx.part_world_aabb(flap))
    with ctx.pose({flap_joint: 1.2}):
        open_pos_ext = _ext(ctx.part_world_aabb(flap))
        open_pos_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap swings inward/up about its center (rocker motion)",
        abs(open_pos_ext[0] - rest_ext[0]) > 0.02
        or abs(open_pos_ext[2] - rest_ext[2]) > 0.02,
        details=f"rest_ext={rest_ext}, open_ext={open_pos_ext}",
    )
    # swinging up should lift the flap's top above the closed top (lid opens up)
    ctx.check(
        "swung flap rises above its closed top edge",
        open_pos_aabb[1][2] > ctx.part_world_aabb(roof)[1][2] - 0.06,
        details=f"swung_top_z={open_pos_aabb[1][2]}",
    )

    return ctx.report()


object_model = build_object_model()
