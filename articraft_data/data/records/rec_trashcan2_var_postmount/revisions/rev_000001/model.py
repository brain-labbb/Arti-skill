from __future__ import annotations

# Post-mounted street trash can: same small cylindrical black plastic body
# with silver domed lid and teardrop swing flap, but hanging from a single
# vertical steel street post.  A pole rises from a ground base plate; a
# bracket cradle (two flat arms + partial-ring clamp bands) clamps around
# the can and holds it elevated above the pavement, leaving an open gap
# between the can floor and the ground.
#
# The trash mouth behind the swing flap remains a genuine open void: the
# body is a hollow open-top shell and the lid has a teardrop through-hole.
#
# Coordinate convention:
#   +Z is up, ground at z = 0.
#   Post is offset along -X from the can centre.
#   Swing flap long axis along X; rocker pivot is horizontal Y through the
#   flap centre.
#
# Root / support:  post  (steel pole + base plate + bolts + bracket arms)
# Primary articulation:  lid_to_flap  REVOLUTE about Y.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    BoxGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- can body dimensions (identical to parent) --------------------------------
R_BOTTOM = 0.105            # body radius at the floor
R_TOP = 0.122              # body radius at the mouth
BODY_H = 0.250             # body height
WALL_T = 0.004
FLOOR_T = 0.008

LID_OVERHANG = 0.010
LID_R = R_TOP + LID_OVERHANG
LID_RISE = 0.060
LID_SKIRT = 0.022

FLAP_LEN = 0.130
FLAP_WID = 0.095
FLAP_RISE = 0.022

LID_TOP_Z = BODY_H + LID_RISE

# ---- post & mounting dimensions ---------------------------------------------
POST_R = 0.025              # steel pole radius
POST_H = 0.90               # pole height
BASE_SIZE = 0.22            # base plate side
BASE_T = 0.012              # base plate thickness
CAN_BOTTOM_Z = 0.55         # can floor elevation above ground
POST_X = -0.25              # post centre X (behind the can)

ARM_W = 0.030               # bracket arm vertical width
ARM_T = 0.006               # bracket arm / band thickness
BAND_H = 0.035              # clamp band height
BOLT_OFFSET = BASE_SIZE * 0.38  # bolt inset from plate edge

# Bracket heights (local to the can body, i.e. height within the bin)
BRACKET_LOCAL_Z = (0.06, BODY_H - 0.06)


# ---- shared helpers ----------------------------------------------------------

def _can_r_at(local_z: float) -> float:
    """Can body outer radius at a given local height.

    Matches the three-point outer shell profile:
      (R_BOTTOM, 0), (R_BOTTOM+0.004, 0.010), (R_TOP, BODY_H)
    """
    if local_z <= 0.010:
        t = local_z / 0.010 if 0.010 > 0 else 0.0
        return R_BOTTOM + 0.004 * t
    t = (local_z - 0.010) / (BODY_H - 0.010)
    return (R_BOTTOM + 0.004) + (R_TOP - R_BOTTOM - 0.004) * t


def _arm_length(can_r: float) -> float:
    """Bracket arm reach from post surface toward the clamp band.

    The arm stops 2 mm short of the body outer surface so the tapered body
    does not intersect the flat arm end face.  The clamp band (at the body
    radius) provides the bracket-to-body contact instead.
    """
    return abs(POST_X) - POST_R - can_r - 0.002


# ---- geometry helpers --------------------------------------------------------

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

    grid_out = []
    grid_in = []
    for i in range(rings + 1):
        f = i / rings
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
    last = rings
    for j in range(seg):
        jn = (j + 1) % seg
        a, b = out_idx[last][j], out_idx[last][jn]
        c, d = in_idx[last][jn], in_idx[last][j]
        geo.add_face(a, b, c); geo.add_face(a, c, d)

    # Rim seat ring: flat annular ring at the body rim height that rests on the
    # body top edge.  Inner radius dips slightly inside the body outer wall so
    # the lid makes robust geometric contact (small intentional overlap).
    rim_inner = R_TOP - 0.001   # 1 mm inside body outer wall
    rim_outer = cr               # matches dome / skirt outer edge
    rim_z_bot = rim_z - 0.003   # thin seat just below the dome edge
    for j in range(seg):
        th0 = 2 * math.pi * j / seg
        th1 = 2 * math.pi * ((j + 1) % seg) / seg
        c0, s0 = math.cos(th0), math.sin(th0)
        c1, s1 = math.cos(th1), math.sin(th1)
        # top face vertices (at rim_z)
        ti0 = geo.add_vertex(rim_inner * c0, rim_inner * s0, rim_z)
        ti1 = geo.add_vertex(rim_inner * c1, rim_inner * s1, rim_z)
        to0 = geo.add_vertex(rim_outer * c0, rim_outer * s0, rim_z)
        to1 = geo.add_vertex(rim_outer * c1, rim_outer * s1, rim_z)
        geo.add_face(ti0, to0, to1); geo.add_face(ti0, to1, ti1)
        # bottom face
        bi0 = geo.add_vertex(rim_inner * c0, rim_inner * s0, rim_z_bot)
        bi1 = geo.add_vertex(rim_inner * c1, rim_inner * s1, rim_z_bot)
        bo0 = geo.add_vertex(rim_outer * c0, rim_outer * s0, rim_z_bot)
        bo1 = geo.add_vertex(rim_outer * c1, rim_outer * s1, rim_z_bot)
        geo.add_face(bi0, bo1, bo0); geo.add_face(bi0, bi1, bo1)
        # outer edge (connects to skirt top)
        geo.add_face(to0, bo0, bo1); geo.add_face(to0, bo1, to1)
        # inner edge
        geo.add_face(ti0, ti1, bi1); geo.add_face(ti0, bi1, bi0)

    # vertical skirt sleeve (open-ended, no end caps)
    skirt = CylinderGeometry(cr, LID_SKIRT, radial_segments=seg, closed=False)
    skirt.translate(0.0, 0.0, rim_z - LID_SKIRT / 2.0)
    geo.merge(skirt)
    return geo


def _flap_mesh() -> MeshGeometry:
    """Teardrop curved swing flap, centred on its pivot."""
    geo = MeshGeometry()
    seg = 48
    rings = 6

    grid = []
    for i in range(rings + 1):
        f = i / rings
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


def _bolt_mesh() -> MeshGeometry:
    """Small hex bolt: short shaft with hex head (shared helper)."""
    shaft = CylinderGeometry(0.004, 0.014, radial_segments=8)
    head = CylinderGeometry(0.007, 0.005, radial_segments=6)
    head.translate(0.0, 0.0, 0.007 + 0.0025)
    shaft.merge(head)
    return shaft


def _bracket_arm_mesh(arm_len: float) -> MeshGeometry:
    """Flat steel bracket arm bar (shared helper).

    Origin at the post-attachment end, extending along +X.
    Centred in Y and Z.
    """
    bar = BoxGeometry((arm_len, ARM_T, ARM_W))
    bar.translate(arm_len / 2.0, 0.0, 0.0)
    return bar


def _clamp_band_mesh(can_radius: float) -> MeshGeometry:
    """Partial cylindrical clamp band (shared helper).

    Wraps ~160 deg around the back of the can (facing the post, -X).
    Inner surface sits on the can outer wall for bracket-to-body contact.
    Centred at origin with can axis along Z; band centred at z = 0.
    """
    inner_r = can_radius          # contacts body outer surface
    outer_r = inner_r + ARM_T
    half_angle = math.radians(80)
    seg = 24

    geo = MeshGeometry()
    first_ring = None
    prev_ring = None

    for i in range(seg + 1):
        a = (math.pi - half_angle) + 2 * half_angle * i / seg
        ca, sa = math.cos(a), math.sin(a)

        ib = geo.add_vertex(inner_r * ca, inner_r * sa, -BAND_H / 2)
        it = geo.add_vertex(inner_r * ca, inner_r * sa,  BAND_H / 2)
        ob = geo.add_vertex(outer_r * ca, outer_r * sa, -BAND_H / 2)
        ot = geo.add_vertex(outer_r * ca, outer_r * sa,  BAND_H / 2)

        curr = (ib, it, ob, ot)
        if i == 0:
            first_ring = curr
        else:
            pib, pit, pob, pot = prev_ring
            # outer face
            geo.add_face(pob, ob, ot)
            geo.add_face(pob, ot, pot)
            # inner face
            geo.add_face(pib, it, ib)
            geo.add_face(pib, pit, it)
            # top
            geo.add_face(pit, pot, ot)
            geo.add_face(pit, ot, it)
            # bottom
            geo.add_face(pib, ob, pob)
            geo.add_face(pib, ib, ob)
        prev_ring = curr

    # end caps
    if first_ring and prev_ring:
        fib, fit, fob, fot = first_ring
        geo.add_face(fib, fob, fot)
        geo.add_face(fib, fot, fit)
        eib, eit, eob, eot = prev_ring
        geo.add_face(eib, eot, eob)
        geo.add_face(eib, eit, eot)

    return geo


# =============================================================================
# build
# =============================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="post_mount_trash_can")

    # materials
    black = model.material("can_black", rgba=(0.10, 0.10, 0.11, 1.0))
    silver = model.material("lid_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    silver_dark = model.material("flap_silver", rgba=(0.66, 0.68, 0.71, 1.0))
    steel = model.material("post_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    dark_steel = model.material("bracket_steel", rgba=(0.35, 0.37, 0.40, 1.0))
    bolt_metal = model.material("bolt_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    # ---- post (root) --------------------------------------------------------
    post = model.part("post")

    # steel pole
    post.visual(
        mesh_from_geometry(
            CylinderGeometry(POST_R, POST_H, radial_segments=32), "post_pole"),
        material=steel,
        name="post_pole",
        origin=Origin(xyz=(POST_X, 0.0, POST_H / 2.0)),
    )

    # post cap
    cap_geo = CylinderGeometry(POST_R + 0.005, 0.008, radial_segments=32)
    cap_geo.translate(0.0, 0.0, POST_H + 0.004)
    post.visual(
        mesh_from_geometry(cap_geo, "post_cap"),
        material=steel,
        name="post_cap",
        origin=Origin(xyz=(POST_X, 0.0, 0.0)),
    )

    # base plate
    post.visual(
        Box((BASE_SIZE, BASE_SIZE, BASE_T)),
        material=steel,
        name="base_plate",
        origin=Origin(xyz=(POST_X, 0.0, BASE_T / 2.0)),
    )

    # base-plate bolts (repeated via for-i-in-range)
    bolt_corners = [
        (POST_X + BOLT_OFFSET,  BOLT_OFFSET),
        (POST_X + BOLT_OFFSET, -BOLT_OFFSET),
        (POST_X - BOLT_OFFSET,  BOLT_OFFSET),
        (POST_X - BOLT_OFFSET, -BOLT_OFFSET),
    ]
    for i in range(4):
        bx, by = bolt_corners[i]
        post.visual(
            mesh_from_geometry(_bolt_mesh(), f"bolt_{i}"),
            material=bolt_metal,
            name=f"bolt_{i}",
            origin=Origin(xyz=(bx, by, BASE_T)),
        )

    # bracket arms + clamp bands (repeated via for-i-in-range)
    for i in range(2):
        local_z = BRACKET_LOCAL_Z[i]
        world_z = CAN_BOTTOM_Z + local_z
        can_r = _can_r_at(local_z)
        arm_len = _arm_length(can_r)

        # flat arm from post surface into the clamp band
        post.visual(
            mesh_from_geometry(_bracket_arm_mesh(arm_len), f"bracket_arm_{i}"),
            material=dark_steel,
            name=f"bracket_arm_{i}",
            origin=Origin(xyz=(POST_X + POST_R, 0.0, world_z)),
        )

        # partial-ring clamp band around the can
        post.visual(
            mesh_from_geometry(_clamp_band_mesh(can_r), f"clamp_band_{i}"),
            material=dark_steel,
            name=f"clamp_band_{i}",
            origin=Origin(xyz=(0.0, 0.0, world_z)),
        )

    # ---- body (FIXED to post, elevated) ------------------------------------
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
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, CAN_BOTTOM_Z)),
    )

    # ---- lid (FIXED to body) -----------------------------------------------
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

    # ---- swing flap (REVOLUTE on lid) --------------------------------------
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

    post = object_model.get_part("post")
    body = object_model.get_part("body")
    lid  = object_model.get_part("lid")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("lid_to_flap")

    # ---- post grounded and correct height ------------------------------------
    post_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "post base sits at ground level",
        post_aabb is not None and post_aabb[0][2] < 0.02,
        details=f"post_min_z={None if post_aabb is None else post_aabb[0][2]}",
    )
    pext = _ext(post_aabb)
    ctx.check(
        "post height ~0.90 m",
        abs(pext[2] - POST_H) < 0.05,
        details=f"post_z_ext={pext[2]}",
    )

    # ---- can is elevated above the ground -----------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "can body elevated well above ground",
        body_aabb[0][2] > 0.35,
        details=f"body_min_z={body_aabb[0][2]}",
    )
    ctx.check(
        "can floor near target elevation",
        abs(body_aabb[0][2] - CAN_BOTTOM_Z) < 0.03,
        details=f"body_min_z={body_aabb[0][2]}, target={CAN_BOTTOM_Z}",
    )

    # open gap between can floor and the base plate (ground reference)
    ctx.expect_gap(
        body, post, axis="z", min_gap=0.30,
        positive_elem="body_shell", negative_elem="base_plate",
        name="open gap between can floor and ground",
    )

    # ---- body dimensions (unchanged from parent) ----------------------------
    bext = _ext(body_aabb)
    ctx.check(
        "body height ~0.25 m",
        abs(bext[2] - BODY_H) < 0.01,
        details=f"body_z_ext={bext[2]}",
    )
    ctx.check(
        "body diameter ~0.24 m",
        0.20 < bext[0] < 0.27 and 0.20 < bext[1] < 0.27,
        details=f"body_xy_ext=({bext[0]},{bext[1]})",
    )

    # ---- dome lid -----------------------------------------------------------
    laabb = ctx.part_world_aabb(lid)
    lext = _ext(laabb)
    ctx.check(
        "dome lid crown rises above body rim",
        laabb[1][2] > body_aabb[1][2] + LID_RISE * 0.5,
        details=f"lid_top={laabb[1][2]}, body_top={body_aabb[1][2]}",
    )
    ctx.check(
        "dome lid overhangs body rim",
        lext[0] > bext[0] + 0.005 and lext[1] > bext[1] + 0.005,
        details=f"lid_ext=({lext[0]},{lext[1]}), body_ext=({bext[0]},{bext[1]})",
    )
    ctx.allow_overlap(
        body, lid,
        elem_a="body_shell", elem_b="lid_dome",
        reason="Lid skirt sleeves over the body rim and the rim seat ring embeds 1 mm "
               "into the body top wall for a seated cap fit.",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.18,
        name="dome lid seats over body mouth footprint",
    )

    # ---- swing flap ---------------------------------------------------------
    faabb = ctx.part_world_aabb(flap)
    fext = _ext(faabb)
    ctx.check(
        "swing flap teardrop ~0.13 x 0.095 m",
        abs(fext[0] - FLAP_LEN) < 0.02 and abs(fext[1] - FLAP_WID) < 0.02,
        details=f"flap_ext=({fext[0]},{fext[1]})",
    )
    ctx.check(
        "swing flap centred on dome crown",
        abs((faabb[0][0] + faabb[1][0]) / 2.0) < 0.02
        and abs((faabb[0][1] + faabb[1][1]) / 2.0) < 0.02,
    )
    ctx.allow_overlap(
        flap, lid,
        reason="Flap sits in the dome opening and overlaps the recess lip at its pivot seam.",
    )
    ctx.expect_contact(flap, lid, name="swing flap seated in dome opening")

    # ---- bracket clamp fit on body -----------------------------------------
    # The clamp bands wrap tightly around the tapered can body; at the wider
    # end of each band there is a slight interference fit (realistic for a
    # clamp cradle).  Scope the allowance to the clamp-band elements.
    for i in range(2):
        ctx.allow_overlap(
            post, body,
            elem_a=f"clamp_band_{i}", elem_b="body_shell",
            reason=f"Clamp band {i} wraps tightly around the tapered body; "
                   "slight interference at the wider end is a realistic clamp fit.",
        )

    # ---- primary articulation: rocker swing about Y -------------------------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ctx.check(
        "flap swing axis is lateral (Y)",
        abs(flap_joint.axis[1]) > 0.99
        and abs(flap_joint.axis[0]) < 0.01
        and abs(flap_joint.axis[2]) < 0.01,
        details=f"axis={flap_joint.axis}",
    )
    lim = flap_joint.motion_limits
    ctx.check(
        "flap travel >= ~90 deg each way",
        lim is not None and lim.upper >= 1.4 and lim.lower <= -1.4,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # rocker: tipping shrinks the X footprint and drops the leading edge
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.3}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap rocks: footprint shrinks in X",
        (open_aabb[1][0] - open_aabb[0][0])
        < (rest_aabb[1][0] - rest_aabb[0][0]) - 0.02,
    )
    ctx.check(
        "flap swing lowers leading edge (opens mouth)",
        open_aabb[0][2] < rest_aabb[0][2] - 0.02,
    )

    # ---- bracket bridges post to can ----------------------------------------
    ctx.expect_overlap(
        post, body, axes="xy", min_overlap=0.02,
        name="bracket arms bridge post to can in XY",
    )

    # bracket clamp bands contact the body surface
    for i in range(2):
        ctx.expect_contact(
            post, body,
            elem_a=f"clamp_band_{i}", elem_b="body_shell",
            name=f"clamp band {i} contacts can body",
        )

    # post rises past the can top (full street post)
    ctx.check(
        "post extends above can top",
        post_aabb[1][2] > CAN_BOTTOM_Z + BODY_H + 0.05,
        details=f"post_top={post_aabb[1][2]}, can_top={CAN_BOTTOM_Z + BODY_H}",
    )

    return ctx.report()


object_model = build_object_model()
