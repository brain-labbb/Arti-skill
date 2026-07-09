from __future__ import annotations

"""MK2-style 'pineapple' fragmentation hand grenade.

Reference image: picture/Military/Granade/001.png

- Olive cast-iron ovoid body with 5 rows x 8 columns of beveled fragmentation
  knobs over a recessed core (waffle grid), flatter rounded base grounded at z=0.
- Yellow/ochre painted band around the narrow neck.
- Polished steel fuze: threaded collar + rectangular pivot housing with a
  pivot lug, axle, and a cross-drilled pin hole.
- Articulations:
    1. safety_lever  REVOLUTE  horizontal -Y axis at the top pivot lug, 0..~90 deg
       (long arm swings outward/up away from the body, hook end dips).
    2. safety_pin    PRISMATIC along +Y, 0..0.020 m extraction from the housing.
    3. pull_ring     REVOLUTE  about the pin shaft (+Y) on the pin head eye,
       -1.2..0.9 rad (~120 deg dangle), child of the pin so it extracts with it.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Dimensional plan (meters)
# ----------------------------------------------------------------------------
BODY_TOP_Z = 0.085          # top of cast body neck
COLLAR_Z0, COLLAR_Z1 = 0.0870, 0.0950
COLLAR_R = 0.0105
HOUSING_HX, HOUSING_HY = 0.007, 0.004   # half extents of fuze pivot housing
HOUSING_Z0, HOUSING_Z1 = 0.0940, 0.1140
PIN_Z = 0.1035              # pull-pin axis height
PIN_HOLE_R = 0.0024
PIN_SHAFT_R = 0.00245   # slight interference fit so the pin truly rides the bore
PIN_TRAVEL = 0.020
LEVER_PIVOT = (-0.0078, 0.1126)   # (x, z) of lever hinge axle
AXLE_R = 0.00175        # slight interference fit inside the lever ear holes
EAR_HOLE_R = 0.0017
LEVER_W_HALF = 0.0058       # lever strip half width along Y
LEVER_T_HALF = 0.0008       # lever strip half thickness
RING_R = 0.0130             # ring centerline radius (outer dia ~0.030)
RING_TUBE_R = 0.0021
RING_JOINT_Y = 0.00825      # ring pivot location along pin axis (pin-local)

# Outer (full) body lathe profile: (radius, z), base flat at z=0.
BODY_PROFILE = [
    (0.0140, 0.000),
    (0.0240, 0.006),
    (0.0282, 0.016),
    (0.0298, 0.028),
    (0.0300, 0.040),
    (0.0285, 0.052),
    (0.0252, 0.064),
    (0.0205, 0.072),
    (0.0160, 0.079),
    (0.0132, 0.0825),
    (0.0130, BODY_TOP_Z),
]

# Recessed waffle-groove core profile (full radius at base ring and neck).
CORE_PROFILE = [
    (0.0140, 0.000),
    (0.0200, 0.004),
    (0.0235, 0.012),
    (0.0250, 0.022),
    (0.0262, 0.034),
    (0.0263, 0.044),
    (0.0248, 0.054),
    (0.0215, 0.064),
    (0.0180, 0.071),
    (0.0160, 0.0765),
    (0.0140, 0.081),
    (0.0130, 0.0835),
    (0.0130, BODY_TOP_Z),
]

KNOB_ROWS_Z = [0.0128, 0.0264, 0.0400, 0.0536, 0.0672]
KNOB_COLS = 8
KNOB_COL_OFFSET_DEG = 22.5   # keep the lever meridian (theta=0) between columns
KNOB_H = 0.0111              # meridional height
KNOB_T = 0.0060              # radial thickness
KNOB_GAP = 0.0040            # tangential groove width between knobs
KNOB_CHAMFER = 0.0012

# Safety-lever centerline in world XZ, from the hook over the fuze top down
# the +X flank of the body (hugging it with a 1-3 mm running gap).
LEVER_PTS_WORLD = [
    (-0.0108, 0.1126),   # hook tip, at pivot height beside the housing lug
    (-0.0102, 0.1150),   # hook upper bend
    (0.0080, 0.1150),    # across the housing top
    (0.0093, 0.1070),    # down the +X housing face
    (0.0120, 0.0940),    # past the collar
    (0.0175, 0.0845),    # past the yellow neck band
    (0.0210, 0.0770),    # onto the body shoulder
    (0.0270, 0.0680),
    (0.0305, 0.0585),
    (0.0327, 0.0485),
    (0.0333, 0.0405),    # spoon tail about 60% down the body
]


def _interp_radius(profile: list[tuple[float, float]], z: float) -> float:
    if z <= profile[0][1]:
        return profile[0][0]
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return profile[-1][0]


def _surface_normal(z: float) -> tuple[float, float]:
    """Outward unit normal (nx, nz) of the body surface at height z."""
    eps = 0.002
    dr = _interp_radius(BODY_PROFILE, z + eps) - _interp_radius(BODY_PROFILE, z - eps)
    dz = 2.0 * eps
    n = (dz, -dr)
    mag = math.hypot(*n)
    return (n[0] / mag, n[1] / mag)


def _lathe(profile: list[tuple[float, float]]) -> cq.Workplane:
    wp = cq.Workplane("XZ").moveTo(0.0, 0.0)
    for r, z in profile:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, profile[-1][1]).close()
    return wp.revolve(360.0, (0.0, 0.0), (0.0, 1.0))


def _body_shape() -> cq.Compound:
    """Recessed core plus 5x8 beveled fragmentation knobs (waffle grid)."""
    solids = [_lathe(CORE_PROFILE).val()]
    for z_c in KNOB_ROWS_Z:
        r_c = _interp_radius(BODY_PROFILE, z_c)
        nx, nz = _surface_normal(z_c)
        tilt_deg = math.degrees(math.atan2(-nz, nx))
        width = 2.0 * math.pi * r_c / KNOB_COLS - KNOB_GAP
        cx = r_c - 0.0025 * nx
        cz = z_c - 0.0025 * nz
        knob = (
            cq.Workplane("XY")
            .box(KNOB_T, width, KNOB_H)
            .edges()
            .chamfer(KNOB_CHAMFER)
            .val()
        )
        knob = knob.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), tilt_deg)
        knob = knob.translate(cq.Vector(cx, 0.0, cz))
        for k in range(KNOB_COLS):
            theta = KNOB_COL_OFFSET_DEG + 45.0 * k
            solids.append(knob.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), theta))
    return cq.Compound.makeCompound(solids)


def _fuze_shape() -> cq.Compound:
    """Threaded collar, pivot housing (pin hole, top chamfer), lug, and axle."""
    collar = cq.Solid.makeCylinder(
        COLLAR_R, COLLAR_Z1 - COLLAR_Z0, cq.Vector(0, 0, COLLAR_Z0), cq.Vector(0, 0, 1)
    )
    housing = (
        cq.Workplane("XY")
        .box(2 * HOUSING_HX, 2 * HOUSING_HY, HOUSING_Z1 - HOUSING_Z0)
        .edges(">Z")
        .chamfer(0.0008)
        .translate((0.0, 0.0, (HOUSING_Z0 + HOUSING_Z1) / 2.0))
    )
    pin_hole = cq.Solid.makeCylinder(
        PIN_HOLE_R, 0.04, cq.Vector(0.0, -0.02, PIN_Z), cq.Vector(0, 1, 0)
    )
    housing = housing.cut(pin_hole)
    # Lug carrying the lever axle just outboard of the housing -X face.
    lug = (
        cq.Workplane("XY")
        .box(0.0040, 2 * HOUSING_HY, 0.0040)
        .translate((-0.00765, 0.0, 0.1126))
    )
    axle = cq.Solid.makeCylinder(
        AXLE_R, 0.0136,
        cq.Vector(LEVER_PIVOT[0], -0.0068, LEVER_PIVOT[1]),
        cq.Vector(0, 1, 0),
    )
    return cq.Compound.makeCompound([collar, housing.val(), lug.val(), axle])


def _strip_polygon(pts: list[tuple[float, float]], half_t: float) -> list[tuple[float, float]]:
    """Offset a polyline into a closed thin-strip polygon (in-plane)."""
    normals: list[tuple[float, float]] = []
    seg_n: list[tuple[float, float]] = []
    for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
        dx, dz = x1 - x0, z1 - z0
        mag = math.hypot(dx, dz)
        seg_n.append((-dz / mag, dx / mag))
    for i in range(len(pts)):
        if i == 0:
            n = seg_n[0]
        elif i == len(pts) - 1:
            n = seg_n[-1]
        else:
            nx = seg_n[i - 1][0] + seg_n[i][0]
            nz = seg_n[i - 1][1] + seg_n[i][1]
            mag = math.hypot(nx, nz) or 1.0
            n = (nx / mag, nz / mag)
        normals.append(n)
    outer = [(p[0] + n[0] * half_t, p[1] + n[1] * half_t) for p, n in zip(pts, normals)]
    inner = [(p[0] - n[0] * half_t, p[1] - n[1] * half_t) for p, n in zip(pts, normals)]
    return outer + inner[::-1]


def _lever_shape() -> cq.Compound:
    """Curved spoon strip plus two pivot ear plates with axle holes (lever-local)."""
    px, pz = LEVER_PIVOT
    local_pts = [(x - px, z - pz) for x, z in LEVER_PTS_WORLD]
    poly = _strip_polygon(local_pts, LEVER_T_HALF)
    wp = cq.Workplane("XZ").moveTo(*poly[0])
    for p in poly[1:]:
        wp = wp.lineTo(*p)
    strip = wp.close().extrude(LEVER_W_HALF, both=True)

    ears = []
    for sign in (1.0, -1.0):
        y_c = sign * 0.0052
        plate = (
            cq.Workplane("XY")
            .box(0.008, 0.0012, 0.0060)
            .translate((0.0, y_c, 0.0))
        )
        hole = cq.Solid.makeCylinder(
            EAR_HOLE_R, 0.002, cq.Vector(0.0, y_c - 0.001, 0.0), cq.Vector(0, 1, 0)
        )
        ears.append(plate.cut(hole).val())
    return cq.Compound.makeCompound([strip.val()] + ears)


def _pin_shape() -> cq.Compound:
    """Cotter-style pull pin: shaft, two retaining legs, head disc (pin-local)."""
    shaft = cq.Solid.makeCylinder(
        PIN_SHAFT_R, 0.0130, cq.Vector(0.0, -0.0030, 0.0), cq.Vector(0, 1, 0)
    )
    legs = [
        cq.Solid.makeCylinder(
            0.0010, 0.0095, cq.Vector(0.0, -0.0115, sz * 0.0011), cq.Vector(0, 1, 0)
        )
        for sz in (1.0, -1.0)
    ]
    head = cq.Solid.makeCylinder(
        0.0035, 0.0035, cq.Vector(0.0, 0.0065, 0.0), cq.Vector(0, 1, 0)
    )
    return cq.Compound.makeCompound([shaft, head] + legs)


def _ring_shape() -> cq.Solid:
    """Pull ring torus, threaded through the pin-head eye, tipped 45 deg up/out."""
    off = RING_R / math.sqrt(2.0)
    return cq.Solid.makeTorus(
        RING_R, RING_TUBE_R, cq.Vector(off, 0.0, off), cq.Vector(0, 1, 0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mk2_pineapple_fragmentation_grenade")

    olive = model.material("cast_iron_olive", rgba=(0.27, 0.32, 0.20, 1.0))
    ochre = model.material("painted_ochre_band", rgba=(0.82, 0.56, 0.10, 1.0))
    steel = model.material("polished_fuze_steel", rgba=(0.74, 0.75, 0.78, 1.0))
    lever_green = model.material("lever_dark_green", rgba=(0.15, 0.19, 0.12, 1.0))
    stainless = model.material("stainless_pin", rgba=(0.80, 0.81, 0.84, 1.0))

    body = model.part("grenade_body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "grenade_body", tolerance=0.0004),
        material=olive,
        name="frag_body",
    )
    body.visual(
        Cylinder(radius=0.0145, length=0.0070),
        origin=Origin(xyz=(0.0, 0.0, 0.0845)),
        material=ochre,
        name="neck_band",
    )
    body.visual(
        mesh_from_cadquery(_fuze_shape(), "fuze_assembly", tolerance=0.0002),
        material=steel,
        name="fuze_assembly",
    )

    lever = model.part("safety_lever")
    lever.visual(
        mesh_from_cadquery(_lever_shape(), "safety_lever", tolerance=0.0002),
        material=lever_green,
        name="spoon",
    )

    pin = model.part("safety_pin")
    pin.visual(
        mesh_from_cadquery(_pin_shape(), "safety_pin", tolerance=0.0002),
        material=stainless,
        name="cotter_pin",
    )

    ring = model.part("pull_ring")
    ring.visual(
        mesh_from_cadquery(_ring_shape(), "pull_ring", tolerance=0.0002),
        material=stainless,
        name="ring_torus",
    )

    # (1) Safety lever: horizontal pivot at the fuze-top lug. The closed long
    # arm runs down +X; axis -Y makes positive q swing it outward and up.
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(LEVER_PIVOT[0], 0.0, LEVER_PIVOT[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.571, effort=5.0, velocity=4.0),
    )

    # (2) Pull pin: slides along its own +Y axis out of the housing bore.
    model.articulation(
        "pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin,
        origin=Origin(xyz=(0.0, 0.0, PIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=PIN_TRAVEL, effort=20.0, velocity=0.5),
    )

    # (3) Pull ring: dangles/swings about the pin shaft axis at the head eye.
    model.articulation(
        "ring_swivel",
        ArticulationType.REVOLUTE,
        parent=pin,
        child=ring,
        origin=Origin(xyz=(0.0, RING_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=0.9, effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("grenade_body")
    lever = object_model.get_part("safety_lever")
    pin = object_model.get_part("safety_pin")
    ring = object_model.get_part("pull_ring")
    lever_j = object_model.get_articulation("lever_pivot")
    pin_j = object_model.get_articulation("pin_slide")
    ring_j = object_model.get_articulation("ring_swivel")

    # The pull ring is threaded through the pin-head eye: intentional local
    # embedding of the ring tube with the head disc.
    ctx.allow_overlap(ring, pin, reason="pull ring threaded through the pin head eye")
    # Light interference fits that carry the moving parts on the fixed fuze:
    # pin shaft riding the housing bore, lever ear holes riding the pivot axle.
    ctx.allow_overlap(
        pin,
        body,
        elem_a=pin.get_visual("cotter_pin"),
        elem_b=body.get_visual("fuze_assembly"),
        reason="pin shaft seated in the housing bore with a light interference fit",
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a=lever.get_visual("spoon"),
        elem_b=body.get_visual("fuze_assembly"),
        reason="lever pivot ears ride on the fuze lug axle with a light interference fit",
    )

    # --- Joint plan: types and ranges -------------------------------------
    assert lever_j.articulation_type == ArticulationType.REVOLUTE
    assert pin_j.articulation_type == ArticulationType.PRISMATIC
    assert ring_j.articulation_type == ArticulationType.REVOLUTE
    assert abs(lever_j.motion_limits.upper - 1.571) < 1e-6  # ~90 deg release
    assert abs(pin_j.motion_limits.upper - 0.020) < 1e-9    # ~0.02 m extraction
    ring_span = ring_j.motion_limits.upper - ring_j.motion_limits.lower
    assert abs(ring_span - 2.1) < 1e-6                      # ~120 deg dangle

    # --- Closed pose: true scale, grounding, assembly seating -------------
    with ctx.pose({lever_j: 0.0, pin_j: 0.0, ring_j: 0.0}):
        bb = ctx.part_world_aabb(body)
        assert bb is not None
        (bx0, by0, bz0), (bx1, by1, bz1) = bb
        assert -1e-6 <= bz0 <= 0.0010, f"base not grounded: zmin={bz0}"
        assert 0.110 <= bz1 <= 0.120, f"body+fuze height {bz1}"      # ~0.115 m tall
        assert 0.056 <= (bx1 - bx0) <= 0.068, f"body dia x {bx1 - bx0}"  # ~0.06 m dia
        assert 0.056 <= (by1 - by0) <= 0.068, f"body dia y {by1 - by0}"

        # Lever hugs the +X flank from the fuze top down past mid-body.
        lb = ctx.part_world_aabb(lever)
        assert lb is not None
        assert lb[0][2] < 0.045, "spoon tail should reach below mid-body"
        assert lb[1][0] > 0.030, "spoon should bow out on the +X flank"
        ctx.expect_overlap(lever, body, axes="z", min_overlap=0.05, name="lever_hugs_body_span")
        ctx.expect_contact(lever, body, contact_tol=0.0035, name="lever_seated_on_fuze")

        # Pin fully inserted through the housing bore, tail proud on -Y.
        pb = ctx.part_world_aabb(pin)
        assert pb is not None
        assert pb[0][1] < -0.010, "cotter legs should protrude on -Y when inserted"
        ctx.expect_contact(pin, body, contact_tol=0.0012, name="pin_seated_in_bore")
        ctx.expect_within(pin, body, axes="x", margin=0.001, name="pin_centered_in_housing")

        # Ring ~0.03 m diameter torus, threaded on the pin head.
        rb = ctx.part_world_aabb(ring)
        assert rb is not None
        assert 0.027 <= (rb[1][0] - rb[0][0]) <= 0.033, "ring outer diameter ~0.03 m"
        assert 0.027 <= (rb[1][2] - rb[0][2]) <= 0.033, "ring outer diameter ~0.03 m"
        ctx.expect_contact(ring, pin, contact_tol=1e-6, name="ring_threaded_on_pin")

    # --- Lever release: off-axis tail swings outward and up ---------------
    with ctx.pose({lever_j: 0.0}):
        closed_lb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_j: 1.4}):
        open_lb = ctx.part_world_aabb(lever)
        assert open_lb is not None and closed_lb is not None
        assert open_lb[1][0] > 0.060, "released spoon tail should swing far out on +X"
        assert open_lb[0][2] > closed_lb[0][2] + 0.030, "released spoon should lift up"

    # --- Pin extraction carries the ring along +Y --------------------------
    with ctx.pose({pin_j: PIN_TRAVEL}):
        pb = ctx.part_world_aabb(pin)
        rb = ctx.part_world_aabb(ring)
        assert pb is not None and rb is not None
        assert pb[0][1] > 0.0045, "extracted pin should clear the housing face"
        assert rb[0][1] > 0.024, "ring should ride out with the pin"

    # --- Ring dangle: off-axis ring center swings about the pin shaft ------
    with ctx.pose({ring_j: 0.0}):
        rb0 = ctx.part_world_aabb(ring)
    with ctx.pose({ring_j: -1.0}):
        rb1 = ctx.part_world_aabb(ring)
        assert rb0 is not None and rb1 is not None
        assert rb1[0][0] < rb0[0][0] - 0.008, "ring should swing toward -X"
    with ctx.pose({ring_j: 0.9}):
        rb2 = ctx.part_world_aabb(ring)
        assert rb2 is not None
        assert rb2[0][2] < rb0[0][2] - 0.005, "ring should dip when swung the other way"

    return ctx.report()


object_model = build_object_model()
