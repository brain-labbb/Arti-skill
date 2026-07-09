from __future__ import annotations

# Square green-painted steel street trash can (reference: a square bin with
# green sheet-metal side panels, dark riveted vertical CORNER POSTS, four short
# FOOTED LEGS, and a tapered pyramidal HOOD on top. The sloped FRONT face of the
# hood carries a recessed "PUSH" swing flap that tips inward when pushed).
#
# Coordinate convention (Z-up world):
#   - up                          : +Z; feet sit at z = 0.
#   - the FRONT (push) face faces : -Y  (the sloped face we push on).
#   - left/right of the bin       : +/-X
#
# Parts / articulations:
#   - bin_body (ROOT): square hollow box of green panels with four dark riveted
#       corner posts, four short feet, and a tapered pyramidal hood whose four
#       faces slope inward to a small flat top; the front hood face has a framed
#       opening for the push flap.
#   - push_flap : the recessed "PUSH" panel set into the sloped front hood face.
#       REVOLUTE about a horizontal axis (along X) at the TOP edge of the flap;
#       a push tips it INWARD (down/back) into the bin, 0..~75 deg.

import math

import cadquery as cq
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
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
HALF = 0.180            # half-width of the square body (full 0.36)
FOOT_H = 0.060          # height of the feet
BODY_Z0 = FOOT_H        # body box bottom (top of feet)
BODY_H = 0.520          # height of the straight square body section
BODY_Z1 = BODY_Z0 + BODY_H   # top of the body / bottom of the hood
HOOD_H = 0.300          # height of the tapered pyramidal hood
HOOD_TOP_HALF = 0.105   # half-width of the small flat hood top
TOP_Z = BODY_Z1 + HOOD_H
WALL = 0.010            # panel thickness
POST = 0.030            # corner post cross-section

# Flap geometry on the FRONT (-Y) sloped hood face.
# The front hood face runs from (y=-HALF, z=BODY_Z1) up to (y=-HOOD_TOP_HALF, z=TOP_Z).
FLAP_W = 0.230          # flap width along X
FLAP_LEN = 0.150        # flap length along the slope


def _front_slope():
    """Return (unit_along_slope, midpoint_outward_normal, slope_angle) for the
    front (-Y) hood face, plus the slope direction vector."""
    # bottom edge of front face at body top, top edge at hood top
    p_bot = (0.0, -HALF, BODY_Z1)
    p_top = (0.0, -HOOD_TOP_HALF, TOP_Z)
    dy = p_top[1] - p_bot[1]   # +Y (inward)
    dz = p_top[2] - p_bot[2]   # +Z (up)
    L = math.hypot(dy, dz)
    along = (0.0, dy / L, dz / L)   # up-the-slope unit (toward top, +Y +Z)
    # outward normal of the front face points -Y and slightly +Z... actually for
    # a face sloping inward as it rises, outward normal is -Y and -Z component.
    # normal = rotate 'along' by -90 about X so it points outward (-Y, +Z? )
    # along=(0, +,+). Outward normal (in YZ) perpendicular & pointing away from
    # axis: n = (0, -dz, +dy)/L  -> (0, -, +) i.e. -Y and +Z. That faces up-out.
    n = (0.0, -dz / L, dy / L)
    return along, n, math.atan2(dz, abs(dy)) if dy != 0 else math.pi / 2, (dy, dz, L)


def _body_panels() -> cq.Workplane:
    """Hollow square body: four green wall panels around an open box, open at the
    top (capped by the hood) and with a floor. Built with CadQuery shell."""
    outer = (
        cq.Workplane("XY")
        .box(2 * HALF, 2 * HALF, BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_Z0))
    )
    # hollow it: remove an inner box leaving walls + floor (open top)
    inner = (
        cq.Workplane("XY")
        .box(2 * (HALF - WALL), 2 * (HALF - WALL), BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_Z0 + WALL))
    )
    return outer.cut(inner)


def _hood_shell() -> MeshGeometry:
    """Tapered pyramidal hood as a hollow frustum (square -> smaller square),
    open at the bottom (sits on the body) and with a small flat top cap.

    The FRONT (-Y) face will receive a framed opening for the push flap, which we
    represent by leaving a recessed cut in that face (the flap part fills it)."""
    geom = MeshGeometry()
    zb, zt = BODY_Z1, TOP_Z
    ho, ht = HALF, HOOD_TOP_HALF          # outer half at bottom / top
    hi_b = HALF - WALL                    # inner half at bottom
    hi_t = HOOD_TOP_HALF - WALL           # inner half at top

    def sq(h, z):
        return [(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)]

    ob = sq(ho, zb); ot = sq(ht, zt)
    ib = sq(hi_b, zb); it = sq(hi_t, zt)

    def add(p):
        return geom.add_vertex(*p)

    obi = [add(p) for p in ob]
    oti = [add(p) for p in ot]
    ibi = [add(p) for p in ib]
    iti = [add(p) for p in it]

    def quad(a, b, c, d):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    # outer slanted faces (outward) - order corners 0..3 ccw; SKIP the front
    # (-Y) face (i==0) -- it is rebuilt below with a real push opening so the
    # flap is not backed by a sealing mesh.
    for i in range(4):
        if i == 0:
            continue
        j = (i + 1) % 4
        quad(obi[i], obi[j], oti[j], oti[i])
    # inner slanted faces (inward winding reversed); SKIP the front (-Y) face.
    for i in range(4):
        if i == 0:
            continue
        j = (i + 1) % 4
        quad(ibi[j], ibi[i], iti[i], iti[j])

    # ---- front (-Y) face rebuilt with the push opening (no sealing mesh) ----
    # The push flap sits in this opening; behind it the face is OPEN so pushing
    # the flap reveals the bin interior instead of a solid sheet.
    def qp(p0, p1, p2, p3, rev=False):
        i0, i1, i2, i3 = add(p0), add(p1), add(p2), add(p3)
        if rev:
            geom.add_face(i0, i3, i2)
            geom.add_face(i0, i2, i1)
        else:
            geom.add_face(i0, i1, i2)
            geom.add_face(i0, i2, i3)

    def ho_s(s):  # outer x half-width (== front -Y offset) at slope fraction s
        return ho + (ht - ho) * s

    def hi_s(s):  # inner x half-width at slope fraction s
        return hi_b + (hi_t - hi_b) * s

    def out_pt(s, x):
        return (x, -ho_s(s), zb + (zt - zb) * s)

    def in_pt(s, x):
        return (x, -hi_s(s), zb + (zt - zb) * s)

    slope_len = math.hypot(ho - ht, zt - zb)
    s_lo = (S_MID - FLAP_LEN / 2.0) / slope_len
    s_hi = (S_MID + FLAP_LEN / 2.0) / slope_len
    xo = FLAP_W / 2.0                     # opening half-width across X

    # outer skin: four border strips framing the rectangular opening
    qp(out_pt(0.0, -ho_s(0.0)), out_pt(0.0, ho_s(0.0)),
       out_pt(s_lo, ho_s(s_lo)), out_pt(s_lo, -ho_s(s_lo)))            # below
    qp(out_pt(s_hi, -ho_s(s_hi)), out_pt(s_hi, ho_s(s_hi)),
       out_pt(1.0, ho_s(1.0)), out_pt(1.0, -ho_s(1.0)))               # above
    qp(out_pt(s_lo, -ho_s(s_lo)), out_pt(s_lo, -xo),
       out_pt(s_hi, -xo), out_pt(s_hi, -ho_s(s_hi)))                  # -X side
    qp(out_pt(s_lo, xo), out_pt(s_lo, ho_s(s_lo)),
       out_pt(s_hi, ho_s(s_hi)), out_pt(s_hi, xo))                    # +X side
    # inner skin: same strips, reversed winding (faces into the bin)
    qp(in_pt(0.0, -hi_s(0.0)), in_pt(0.0, hi_s(0.0)),
       in_pt(s_lo, hi_s(s_lo)), in_pt(s_lo, -hi_s(s_lo)), rev=True)
    qp(in_pt(s_hi, -hi_s(s_hi)), in_pt(s_hi, hi_s(s_hi)),
       in_pt(1.0, hi_s(1.0)), in_pt(1.0, -hi_s(1.0)), rev=True)
    qp(in_pt(s_lo, -hi_s(s_lo)), in_pt(s_lo, -xo),
       in_pt(s_hi, -xo), in_pt(s_hi, -hi_s(s_hi)), rev=True)
    qp(in_pt(s_lo, xo), in_pt(s_lo, hi_s(s_lo)),
       in_pt(s_hi, hi_s(s_hi)), in_pt(s_hi, xo), rev=True)
    # lip around the opening: connect outer hole edge to inner hole edge so the
    # cut reads as real sheet-metal thickness (double-sided for a clean read).
    for (pa, pb) in (
        ((s_lo, -xo), (s_lo, xo)),   # bottom edge of opening
        ((s_hi, xo), (s_hi, -xo)),   # top edge
        ((s_lo, xo), (s_hi, xo)),    # +X edge
        ((s_hi, -xo), (s_lo, -xo)),  # -X edge
    ):
        o0, o1 = out_pt(*pa), out_pt(*pb)
        i0, i1 = in_pt(*pa), in_pt(*pb)
        qp(o0, o1, i1, i0)
        qp(o0, o1, i1, i0, rev=True)
    # bottom rim band (closes wall thickness where hood meets body)
    for i in range(4):
        j = (i + 1) % 4
        quad(obi[j], obi[i], ibi[i], ibi[j])
    # top: flat cap (solid small square top), close outer top to inner top edge
    # ring band:
    for i in range(4):
        j = (i + 1) % 4
        quad(oti[i], oti[j], iti[j], iti[i])
    # solid cap plate across the inner top opening
    c0, c1, c2, c3 = iti
    quad(c0, c1, c2, c3)
    return geom


def _corner_post(sx: float, sy: float) -> MeshGeometry:
    """A dark riveted vertical corner post running the full body height, with a
    column of rivet bumps. Placed at corner (sx*HALF, sy*HALF)."""
    g = MeshGeometry()
    post = BoxGeometry((POST, POST, BODY_H + 0.02))
    post.translate(sx * (HALF - POST / 2 + 0.004), sy * (HALF - POST / 2 + 0.004),
                   BODY_Z0 + (BODY_H) / 2.0)
    g.merge(post)
    # rivet bumps down the outer corner
    n_riv = 7
    for k in range(n_riv):
        z = BODY_Z0 + 0.05 + k * (BODY_H - 0.10) / (n_riv - 1)
        riv = CylinderGeometry(0.006, 0.010, radial_segments=10)
        riv.rotate_x(math.pi / 2.0)  # along Y
        riv.translate(sx * (HALF + 0.001), sy * (HALF - POST / 2 + 0.004), z)
        g.merge(riv)
        riv2 = CylinderGeometry(0.006, 0.010, radial_segments=10)
        riv2.rotate_y(math.pi / 2.0)  # along X
        riv2.translate(sx * (HALF - POST / 2 + 0.004), sy * (HALF + 0.001), z)
        g.merge(riv2)
    return g


def _foot(sx: float, sy: float) -> MeshGeometry:
    """A short flat foot/leg pad under one corner post."""
    g = MeshGeometry()
    leg = BoxGeometry((POST + 0.006, POST + 0.006, FOOT_H))
    leg.translate(sx * (HALF - POST / 2 + 0.004), sy * (HALF - POST / 2 + 0.004), FOOT_H / 2.0)
    g.merge(leg)
    # small flat foot pad at the very bottom
    pad = BoxGeometry((POST + 0.020, POST + 0.020, 0.010))
    pad.translate(sx * (HALF - POST / 2 + 0.004), sy * (HALF - POST / 2 + 0.004), 0.005)
    g.merge(pad)
    return g


def _slope_roll() -> float:
    """Roll about +X that maps a flap-local frame (x=width, +y=up-slope,
    +z=outward) onto the world front sloped face. roll = atan2(along_z, along_y)."""
    along, _, _, _ = _front_slope()
    return math.atan2(along[2], along[1])


# distance up-slope (from the body-top front edge) to the flap-region centre.
S_MID = 0.5 * FLAP_LEN + 0.050


def _flap_frame() -> MeshGeometry:
    """A raised frame (bezel) around the push opening on the front hood face, so
    the recessed flap reads as set into a framed pocket. Built as four bars in
    the flap-local plane, then rolled onto the slope and seated on the face."""
    along, n, _, _ = _front_slope()
    base = (0.0, -HALF, BODY_Z1)
    cx = 0.0
    cy = base[1] + along[1] * S_MID
    cz = base[2] + along[2] * S_MID

    g = MeshGeometry()
    bar_t = 0.016
    half_w = FLAP_W / 2 + 0.014
    half_l = FLAP_LEN / 2 + 0.014
    rail = 0.016
    # local plane: x = width (X), y = up-slope, z = outward
    for vv in (+half_l, -half_l):
        b = BoxGeometry((2 * half_w, rail, bar_t))
        b.translate(0.0, vv, 0.0)
        g.merge(b)
    for uu in (+half_w, -half_w):
        b = BoxGeometry((rail, 2 * half_l, bar_t))
        b.translate(uu, 0.0, 0.0)
        g.merge(b)
    g.rotate_x(_slope_roll())   # +y -> up-slope, +z -> outward normal
    g.translate(cx, cy, cz)
    # seat the frame on the face: half its thickness sits proud, half embeds
    g.translate(n[0] * (bar_t * 0.25), n[1] * (bar_t * 0.25), n[2] * (bar_t * 0.25))
    return g


def _flap_geometry() -> MeshGeometry:
    """The push flap panel, built in its HINGE-LOCAL frame (origin at the TOP
    hinge edge): x = width (X), +y = up-slope, +z = outward. The hinge is at the
    top, so the plate extends DOWN the slope (toward -y) over y in [-FLAP_LEN, 0].
    The outward (+z) face carries an embossed 'PUSH' panel."""
    g = MeshGeometry()
    plate = BoxGeometry((FLAP_W, FLAP_LEN, 0.014))
    plate.translate(0.0, -FLAP_LEN / 2.0, 0.0)
    g.merge(plate)
    # recessed inner panel reading as the dished "PUSH" plate
    inner = BoxGeometry((FLAP_W - 0.040, FLAP_LEN - 0.040, 0.006))
    inner.translate(0.0, -FLAP_LEN / 2.0, 0.008)
    g.merge(inner)
    # embossed "PUSH" suggested by four small raised ribs (letters)
    for i in range(4):
        rib = BoxGeometry((0.022, 0.040, 0.004))
        rib.translate(-0.060 + i * 0.040, -FLAP_LEN / 2.0, 0.011)
        g.merge(rib)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_street_trash_can")

    green = model.material("painted_green", rgba=(0.55, 0.60, 0.20, 1.0))
    green_dark = model.material("painted_green_dark", rgba=(0.42, 0.46, 0.16, 1.0))
    steel = model.material("riveted_steel", rgba=(0.34, 0.30, 0.24, 1.0))
    steel_lt = model.material("steel_light", rgba=(0.46, 0.43, 0.38, 1.0))

    # ---- body (root): green panels + hood + posts + feet ----
    body = model.part("bin_body")
    body.visual(mesh_from_cadquery(_body_panels(), "green_panels"), material=green, name="green_panels")
    body.visual(mesh_from_geometry(_hood_shell(), "hood"), material=green_dark, name="hood")

    for sx, sy, nm in ((1, 1, "ne"), (1, -1, "se"), (-1, 1, "nw"), (-1, -1, "sw")):
        body.visual(mesh_from_geometry(_corner_post(sx, sy), f"post_{nm}"), material=steel, name=f"post_{nm}")
        body.visual(mesh_from_geometry(_foot(sx, sy), f"foot_{nm}"), material=steel, name=f"foot_{nm}")

    body.visual(mesh_from_geometry(_flap_frame(), "push_frame"), material=steel_lt, name="push_frame")

    body.inertial = Inertial.from_geometry(
        Box((2 * HALF, 2 * HALF, TOP_Z)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z / 2.0)),
    )

    # ---- push flap (revolute, hinged at its TOP edge, tips inward) ----
    along, n, _, _ = _front_slope()
    flap = model.part("push_flap")
    flap.visual(mesh_from_geometry(_flap_geometry(), "flap_panel"), material=steel_lt, name="flap_panel")
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_W, FLAP_LEN, 0.02)), mass=0.4,
        origin=Origin(xyz=(0.0, -FLAP_LEN / 2.0, 0.0)),
    )
    # Hinge frame at the TOP edge of the flap region on the front sloped face.
    s_top = S_MID + FLAP_LEN / 2.0
    base = (0.0, -HALF, BODY_Z1)
    hinge_y = base[1] + along[1] * s_top
    hinge_z = base[2] + along[2] * s_top
    # Roll maps the flap-local frame (x=width, +y=up-slope, +z=outward) onto the
    # world slope, so at q=0 the flap lies flush in the front face, plate
    # hanging down-slope from the hinge.
    roll = _slope_roll()
    # Pushing tips the free (down-slope) bottom edge INWARD (+Y into the bin).
    # axis sign verified by the pose test below; +X gives the inward swing.
    model.articulation(
        "body_to_push_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(roll, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(75.0), effort=4.0, velocity=2.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bin_body")
    flap = object_model.get_part("push_flap")
    flap_joint = object_model.get_articulation("body_to_push_flap")

    # ---- feet sit at z ~ 0 ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "feet sit at z~0",
        abs(body_aabb[0][2]) < 0.005,
        details=f"body z-min={body_aabb[0][2]:.4f}",
    )
    # ---- square footprint (x ~ y), measured on the green body panels only
    # (the push frame/flap protrude in front, so use the panel element AABB) ----
    panel_aabb = ctx.part_element_world_aabb(body, elem="green_panels")
    pext = _ext(panel_aabb)
    ctx.check(
        "square footprint (x~y)",
        abs(pext[0] - pext[1]) < 0.03,
        details=f"green panel extents (x,y)={pext[0]:.3f},{pext[1]:.3f}",
    )
    ext = _ext(body_aabb)
    ctx.check(
        "tall bin (~0.8-1.0 m)",
        0.75 < ext[2] < 1.05,
        details=f"body height={ext[2]:.3f}",
    )
    # ---- hood narrows toward the top (pyramidal) ----
    ctx.check(
        "hood is tapered (top narrower than body)",
        HOOD_TOP_HALF < HALF - 0.04,
        details=f"hood_top_half={HOOD_TOP_HALF}, body_half={HALF}",
    )
    # ---- hollow body: an interior cavity beneath the hood ----
    ctx.check(
        "hollow body cavity",
        (BODY_Z1 - (BODY_Z0 + WALL)) > 0.40,
        details=f"cavity depth ~{BODY_Z1 - (BODY_Z0 + WALL):.3f} m",
    )

    # ---- four corner posts and four feet present ----
    for nm in ("ne", "se", "nw", "sw"):
        ctx.check(f"corner post {nm} present", body.get_visual(f"post_{nm}") is not None, details=nm)
        ctx.check(f"foot {nm} present", body.get_visual(f"foot_{nm}") is not None, details=nm)

    # ---- push flap on the FRONT (-Y) face, set into its frame ----
    flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "push flap on the front (-Y) face",
        flap_aabb[0][1] < -0.10,
        details=f"flap y-min={flap_aabb[0][1]:.3f}",
    )
    ctx.check(
        "push flap is up on the hood (above body top)",
        flap_aabb[1][2] > BODY_Z1 - 0.02,
        details=f"flap z-max={flap_aabb[1][2]:.3f}, body_top={BODY_Z1:.3f}",
    )
    # flap seats into the framed pocket (intended local overlap at the bezel)
    ctx.allow_overlap(flap, body, reason="Push flap is recessed into the framed pocket on the sloped hood face.")

    # ---- pushing tips the flap INWARD (free bottom edge moves +Y into the bin) ----
    rest_aabb = ctx.part_world_aabb(flap)
    rest_inner_y = rest_aabb[1][1]    # most +Y (inward) extent at rest
    rest_low_z = rest_aabb[0][2]
    with ctx.pose({flap_joint: math.radians(75.0)}):
        push_aabb = ctx.part_world_aabb(flap)
        push_inner_y = push_aabb[1][1]
    ctx.check(
        "push tips the flap inward (free edge swings into the bin, +Y)",
        push_inner_y > rest_inner_y + 0.04,
        details=f"rest_inner_y={rest_inner_y:.3f}, push_inner_y={push_inner_y:.3f}",
    )
    # hinge axis is horizontal along X (the top edge of the flap)
    ax = flap_joint.axis
    ctx.check(
        "flap hinge axis horizontal along X",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.01 and abs(ax[2]) < 0.01,
        details=f"axis={ax}",
    )

    return ctx.report()


object_model = build_object_model()
