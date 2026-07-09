from __future__ import annotations

# Green steel commercial front-load dumpster with a rear-hinged slatted lid,
# rolling on four swivel casters instead of fixed feet.
#
# Real object: a weathered olive-green steel front-load waste container. The
# body is a tapered rectangular bin (wider at the top than the floor) with
# vertical corrugation ribs pressed into all four walls and a rolled top rim.
# The bin is open inside (hollow). A slatted steel LID caps the top: parallel
# ribs/slats running front-to-back, hinged along the REAR top edge so it flips
# up and back to open. At each bottom corner a short steel mounting plate
# carries a caster fork and a round rubber wheel so the whole bin can be
# wheeled around. Two front forklift/skid pockets and side lifting trunnion
# pockets remain.
#
# Coordinate convention:
#   - +Z is up; the wheel bottoms sit at z = 0.
#   - The truck approaches from the front (+X). The lid hinges along the REAR
#     (-X) top edge and lifts up/back.
#   - Centerline is y = 0; the bin is left/right symmetric across it.
#
# Root structure: the body is the root. Caster forks are inline body visuals.
# The slatted lid is hinged at the rear top edge (REVOLUTE). Each wheel is a
# separate child part with a CONTINUOUS roll joint about its horizontal axle.
#
# Articulations:
#   - body_to_lid      : REVOLUTE, rear-hinged flip lid, axis along Y.
#   - body_to_wheel_i  : CONTINUOUS, wheel roll about horizontal Y axle (i=0..3).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
# A ~2 cubic yard front-load dumpster.
BODY_W = 1.80              # width (Y)
BODY_D_BOT = 1.00         # depth at the floor (X)
BODY_D_TOP = 1.18         # depth at the top (X) -> outward taper toward the top
BODY_H = 1.15             # body wall height (top rim z above the floor reference)
WALL_T = 0.012
FLOOR_T = 0.020

FOOT_H = 0.17             # body floor height above ground (caster clearance)

RIM_T = 0.030             # rolled top rim thickness
RIB_COUNT_SIDE = 9        # vertical corrugation ribs along the long (Y) walls
RIB_COUNT_END = 5         # vertical ribs on the short (X) end walls
RIB_W = 0.045
RIB_PROUD = 0.020         # how far ribs stand proud of the wall
RIB_EMBED = 0.004         # slight embed prevents coplanar z-fighting

LID_SLATS = 11            # number of slats across the lid
LID_GAP = 0.012           # gap between slats
LID_THK = 0.030
LID_OVER = 0.020          # lid overhang past the rim

WALL_TOP_Z = FOOT_H + BODY_H   # top rim z

# ---- caster dimensions -------------------------------------------------------
CASTER_INSET_X = 0.10     # caster inset from body end in X
CASTER_INSET_Y = 0.10     # caster inset from body side in Y
WHEEL_R = 0.060           # wheel radius (120 mm diameter, ~4.7")
WHEEL_W = 0.050           # wheel width (axle direction)
PLATE_SIZE = 0.100        # mounting plate side length
PLATE_T = 0.010           # mounting plate thickness
HOUSING_R = 0.022         # swivel housing half-width (square approx)
HOUSING_H = 0.020         # swivel housing height
CROSSBAR_H = 0.015        # fork crossbar height
FORK_LEG_DX = 0.020       # fork leg depth (X direction)
FORK_LEG_DY = 0.008       # fork leg thickness (Y direction)
FORK_INNER_GAP = WHEEL_W          # fork legs contact the wheel faces (captured fit)
FORK_OUTER_W = FORK_INNER_GAP + 2 * FORK_LEG_DY  # total fork outer width

# Corner positions for casters (in body frame)
CASTER_HX = BODY_D_BOT / 2.0 - CASTER_INSET_X   # ±X position
CASTER_HY = BODY_W / 2.0 - CASTER_INSET_Y        # ±Y position

# Fork local-frame Z positions (mounting face at z=0, fork extends to -Z):
#   plate:     [-PLATE_T, 0]
#   housing:   [-PLATE_T - HOUSING_H, -PLATE_T]
#   crossbar:  [-PLATE_T - HOUSING_H - CROSSBAR_H, -PLATE_T - HOUSING_H]
#   legs:      [leg_bot, crossbar_bot + embed]
_FORK_PLATE_BOT = -PLATE_T
_FORK_HOUSING_BOT = _FORK_PLATE_BOT - HOUSING_H
_FORK_CROSSBAR_BOT = _FORK_HOUSING_BOT - CROSSBAR_H
_FORK_LEG_TOP = _FORK_CROSSBAR_BOT + 0.002   # 2 mm embed into crossbar
_FORK_LEG_BOT = -(FOOT_H - WHEEL_R + 0.010)  # extends 10 mm below axle center
_FORK_LEG_H = _FORK_LEG_TOP - _FORK_LEG_BOT
_FORK_LEG_CY = FORK_INNER_GAP / 2.0 + FORK_LEG_DY / 2.0  # leg center Y offset


def _tapered_body() -> MeshGeometry:
    """Hollow tapered rectangular bin: 4 trapezoidal walls + floor, open top.

    Authored in the body frame with the feet/floor reference; the wall block
    spans z in [FOOT_H, WALL_TOP_Z]. Slightly wider in X at the top (front-load
    taper). Hollow inside.
    """
    geo = MeshGeometry()
    hwb = BODY_W / 2.0
    hwt = BODY_W / 2.0           # width taper is negligible; keep straight in Y
    hdb = BODY_D_BOT / 2.0
    hdt = BODY_D_TOP / 2.0
    z0 = FOOT_H
    z1 = WALL_TOP_Z

    # inner offsets
    hwbi = hwb - WALL_T
    hwti = hwt - WALL_T
    hdbi = hdb - WALL_T
    hdti = hdt - WALL_T

    def quad(p0, p1, p2, p3):
        i0 = geo.add_vertex(*p0); i1 = geo.add_vertex(*p1)
        i2 = geo.add_vertex(*p2); i3 = geo.add_vertex(*p3)
        geo.add_face(i0, i1, i2); geo.add_face(i0, i2, i3)

    # +X (front) wall  -- outer, inner, rim
    quad((hdb, -hwb, z0), (hdt, -hwt, z1), (hdt, hwt, z1), (hdb, hwb, z0))
    quad((hdbi, hwbi, z0), (hdti, hwti, z1), (hdti, -hwti, z1), (hdbi, -hwbi, z0))
    quad((hdt, -hwt, z1), (hdti, -hwti, z1), (hdti, hwti, z1), (hdt, hwt, z1))
    # -X (rear) wall
    quad((-hdb, hwb, z0), (-hdt, hwt, z1), (-hdt, -hwt, z1), (-hdb, -hwb, z0))
    quad((-hdbi, -hwbi, z0), (-hdti, -hwti, z1), (-hdti, hwti, z1), (-hdbi, hwbi, z0))
    quad((-hdt, hwt, z1), (-hdti, hwti, z1), (-hdti, -hwti, z1), (-hdt, -hwt, z1))
    # +Y wall
    quad((hdb, hwb, z0), (hdt, hwt, z1), (-hdt, hwt, z1), (-hdb, hwb, z0))
    quad((-hdbi, hwbi, z0), (-hdti, hwti, z1), (hdti, hwti, z1), (hdbi, hwbi, z0))
    quad((hdt, hwt, z1), (hdti, hwti, z1), (-hdti, hwti, z1), (-hdt, hwt, z1))
    # -Y wall
    quad((-hdb, -hwb, z0), (-hdt, -hwt, z1), (hdt, -hwt, z1), (hdb, -hwb, z0))
    quad((hdbi, -hwbi, z0), (hdti, -hwti, z1), (-hdti, -hwti, z1), (-hdbi, -hwbi, z0))
    quad((-hdt, -hwt, z1), (-hdti, -hwti, z1), (hdti, -hwti, z1), (hdt, -hwt, z1))

    # floor slab just above the feet
    floor = BoxGeometry((BODY_D_BOT - 2 * WALL_T, BODY_W - 2 * WALL_T, FLOOR_T))
    floor.translate(0.0, 0.0, z0 + FLOOR_T / 2.0)
    geo.merge(floor)
    return geo


def _ribs() -> MeshGeometry:
    """Corrugation ribs pressed proud of, and flush to, the tapered walls."""
    geo = MeshGeometry()
    rib_h = BODY_H - 0.10
    z0 = FOOT_H + (BODY_H - rib_h) / 2.0
    z1 = z0 + rib_h
    taper = (BODY_D_TOP - BODY_D_BOT) / (2.0 * BODY_H)

    def half_depth_at_z(z: float) -> float:
        return BODY_D_BOT / 2.0 + taper * (z - FOOT_H)

    def add_prism(inner_loop, outer_loop) -> None:
        indices = [geo.add_vertex(*p) for p in inner_loop + outer_loop]

        def tri(a, b, c):
            geo.add_face(indices[a], indices[b], indices[c])

        # inner face, proud face, then four edge faces
        tri(0, 1, 2); tri(0, 2, 3)
        tri(4, 6, 5); tri(4, 7, 6)
        for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
            tri(a, b, b + 4)
            tri(a, b + 4, a + 4)

    # long (+/-Y) walls: ribs spread across X
    for sgn in (1.0, -1.0):
        span_bot = BODY_D_BOT * 0.86
        span_top = BODY_D_TOP * 0.86
        y_inner = sgn * (BODY_W / 2.0 - RIB_EMBED)
        y_outer = sgn * (BODY_W / 2.0 + RIB_PROUD)
        for i in range(RIB_COUNT_SIDE):
            t = (i + 0.5) / RIB_COUNT_SIDE
            x_bot = -span_bot / 2.0 + span_bot * t
            x_top = -span_top / 2.0 + span_top * t
            inner = [
                (x_bot - RIB_W / 2.0, y_inner, z0),
                (x_bot + RIB_W / 2.0, y_inner, z0),
                (x_top + RIB_W / 2.0, y_inner, z1),
                (x_top - RIB_W / 2.0, y_inner, z1),
            ]
            outer = [(x, y_outer, z) for x, _, z in inner]
            add_prism(inner, outer)
    # short (+/-X) end walls: ribs spread across Y
    for sgn in (1.0, -1.0):
        n_len = math.sqrt(1.0 + taper * taper)
        nx = sgn / n_len
        nz = -taper / n_len
        span = BODY_W * 0.88
        for i in range(RIB_COUNT_END):
            t = (i + 0.5) / RIB_COUNT_END
            y = -span / 2.0 + span * t
            wall_bot = (sgn * half_depth_at_z(z0), y, z0)
            wall_top = (sgn * half_depth_at_z(z1), y, z1)
            inner = [
                (wall_bot[0] - nx * RIB_EMBED, y - RIB_W / 2.0,
                 wall_bot[2] - nz * RIB_EMBED),
                (wall_bot[0] - nx * RIB_EMBED, y + RIB_W / 2.0,
                 wall_bot[2] - nz * RIB_EMBED),
                (wall_top[0] - nx * RIB_EMBED, y + RIB_W / 2.0,
                 wall_top[2] - nz * RIB_EMBED),
                (wall_top[0] - nx * RIB_EMBED, y - RIB_W / 2.0,
                 wall_top[2] - nz * RIB_EMBED),
            ]
            outer = [
                (x + nx * (RIB_PROUD + RIB_EMBED), yv,
                 z + nz * (RIB_PROUD + RIB_EMBED))
                for x, yv, z in inner
            ]
            add_prism(inner, outer)
    return geo


def _rim() -> MeshGeometry:
    """Rolled top rim ring around the mouth."""
    geo = MeshGeometry()
    hd = BODY_D_TOP / 2.0
    hw = BODY_W / 2.0
    z = WALL_TOP_Z
    # four rim bars framing the mouth
    bar_x = BoxGeometry((BODY_D_TOP + 2 * RIM_T, RIM_T * 2, RIM_T))
    for sgn in (1.0, -1.0):
        b = bar_x.clone(); b.translate(0.0, sgn * hw, z - RIM_T / 2.0); geo.merge(b)
    bar_y = BoxGeometry((RIM_T * 2, BODY_W, RIM_T))
    for sgn in (1.0, -1.0):
        b = bar_y.clone(); b.translate(sgn * hd, 0.0, z - RIM_T / 2.0); geo.merge(b)
    return geo


def _caster_fork_mesh() -> MeshGeometry:
    """One caster fork + mounting plate in local frame.

    Mounting face at z=0 (top), fork extends to -Z. When placed at body
    position (cx, cy, FOOT_H), the fork hangs below the body floor and the
    wheel axle center lands at world z = WHEEL_R.
    """
    geo = MeshGeometry()
    embed = 0.002  # small overlap for mesh connectivity

    # Mounting plate: z = [_FORK_PLATE_BOT, 0]
    plate = BoxGeometry((PLATE_SIZE, PLATE_SIZE, PLATE_T))
    plate.translate(0.0, 0.0, _FORK_PLATE_BOT / 2.0)
    geo.merge(plate)

    # Swivel housing: z = [_FORK_HOUSING_BOT, _FORK_PLATE_BOT + embed]
    h_cz = (_FORK_HOUSING_BOT + _FORK_PLATE_BOT + embed) / 2.0
    h_h = (_FORK_PLATE_BOT + embed) - _FORK_HOUSING_BOT
    housing = BoxGeometry((HOUSING_R * 2, HOUSING_R * 2, h_h))
    housing.translate(0.0, 0.0, h_cz)
    geo.merge(housing)

    # Crossbar: z = [_FORK_CROSSBAR_BOT, _FORK_HOUSING_BOT + embed]
    c_cz = (_FORK_CROSSBAR_BOT + _FORK_HOUSING_BOT + embed) / 2.0
    c_h = (_FORK_HOUSING_BOT + embed) - _FORK_CROSSBAR_BOT
    crossbar = BoxGeometry((FORK_LEG_DX, FORK_OUTER_W, c_h))
    crossbar.translate(0.0, 0.0, c_cz)
    geo.merge(crossbar)

    # Fork legs: two parallel prongs extending past the axle center
    leg_cz = (_FORK_LEG_TOP + _FORK_LEG_BOT) / 2.0
    for sy in (1.0, -1.0):
        leg = BoxGeometry((FORK_LEG_DX, FORK_LEG_DY, _FORK_LEG_H))
        leg.translate(0.0, sy * _FORK_LEG_CY, leg_cz)
        geo.merge(leg)

    return geo


def _pockets_and_trunnions() -> MeshGeometry:
    """Front forklift pockets and side lifting trunnion pockets."""
    geo = MeshGeometry()
    # two front (+X) forklift pockets: horizontal tubes across the front base
    for sy in (1.0, -1.0):
        pocket = BoxGeometry((0.22, 0.16, 0.12))
        pocket.translate(BODY_D_BOT / 2.0 - 0.02, sy * BODY_W * 0.26, FOOT_H + 0.06)
        geo.merge(pocket)
    # side lifting trunnion pockets (the truck-arm pockets), one per long wall
    for sy in (1.0, -1.0):
        trunnion = BoxGeometry((0.10, RIB_PROUD * 2.2, 0.16))
        trunnion.translate(BODY_D_TOP * 0.18, sy * (BODY_W / 2.0),
                           FOOT_H + BODY_H * 0.62)
        geo.merge(trunnion)
    return geo


def _lid_mesh() -> MeshGeometry:
    """Slatted steel lid: parallel ribs running front-to-back (X), with gaps.

    Authored in the lid's OWN frame with the hinge axis (Y) at local x=0, z=0
    along the REAR edge: the lid panel extends forward (+X) from the hinge and
    the slat ribs span the full depth. A thin base skin ties all slats together
    so it reads as one connected lid, not floating slats.
    """
    geo = MeshGeometry()
    depth = BODY_D_TOP + 2 * LID_OVER       # front-to-back length of the lid
    width = BODY_W + 2 * LID_OVER           # across the slats (Y)
    slat_pitch = (width - LID_GAP) / LID_SLATS
    slat_w = slat_pitch - LID_GAP
    # base skin tying the slats together
    skin = BoxGeometry((depth, width, 0.008))
    skin.translate(depth / 2.0, 0.0, -0.004)
    geo.merge(skin)
    for i in range(LID_SLATS):
        yc = -width / 2.0 + slat_pitch * (i + 0.5)
        slat = BoxGeometry((depth, slat_w, LID_THK))
        slat.translate(depth / 2.0, yc, LID_THK / 2.0)
        geo.merge(slat)
    # a small front lift handle/lip on the leading (+X) edge
    handle = BoxGeometry((0.05, width * 0.4, 0.05))
    handle.translate(depth - 0.025, 0.0, LID_THK + 0.005)
    geo.merge(handle)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dumpster")

    green = model.material("steel_green", rgba=(0.27, 0.40, 0.20, 1.0))
    green_dark = model.material("steel_green_dark", rgba=(0.20, 0.31, 0.15, 1.0))
    lid_dark = model.material("lid_steel", rgba=(0.16, 0.18, 0.15, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.18, 0.19, 0.18, 1.0))
    rubber = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(mesh_from_geometry(_tapered_body(), "body_shell"),
                material=green, name="body_shell")
    body.visual(mesh_from_geometry(_ribs(), "body_ribs"),
                material=green_dark, name="body_ribs")
    body.visual(mesh_from_geometry(_rim(), "top_rim"),
                material=green_dark, name="top_rim")
    body.visual(mesh_from_geometry(_pockets_and_trunnions(), "base_hardware"),
                material=fork_steel, name="base_hardware")

    # ---- four caster forks (inline body visuals, one per corner) -------------
    corners = [
        (CASTER_HX, CASTER_HY),    # front-right
        (CASTER_HX, -CASTER_HY),   # front-left
        (-CASTER_HX, CASTER_HY),   # rear-right
        (-CASTER_HX, -CASTER_HY),  # rear-left
    ]
    for i, (cx, cy) in enumerate(corners):
        body.visual(
            mesh_from_geometry(_caster_fork_mesh(), f"caster_fork_{i}"),
            origin=Origin(xyz=(cx, cy, FOOT_H)),
            material=fork_steel,
            name=f"caster_fork_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_D_TOP, BODY_W, BODY_H)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + BODY_H * 0.45)),
    )

    # ---- four caster wheels (one child part per corner, continuous roll) ----
    for i, (cx, cy) in enumerate(corners):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            # Cylinder is along local Z; rotate so axle is along world Y.
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="wheel_tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            mass=1.5,
        )
        # Continuous roll joint about the horizontal Y axle at wheel center.
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(cx, cy, WHEEL_R)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=10.0),
        )

    # ---- slatted rear-hinged lid -------------------------------------------
    lid = model.part("lid")
    lid.visual(mesh_from_geometry(_lid_mesh(), "lid_slats"),
               material=lid_dark, name="lid_slats")
    lid.inertial = Inertial.from_geometry(
        Box((BODY_D_TOP, BODY_W, LID_THK)),
        mass=14.0,
        origin=Origin(xyz=(BODY_D_TOP / 2.0, 0.0, 0.0)),
    )
    # Hinge at the REAR top edge. The lid local frame has its hinge at x=0 and
    # the lid extends to +X; placed at the rear (-X) rim so closed it lies flat
    # over the mouth reaching to the front. axis = +Y: positive q lifts the
    # front (+X) edge up and back (a real rear-hinged flip lid).
    hinge_x = -BODY_D_TOP / 2.0 - LID_OVER
    hinge_z = WALL_TOP_Z + 0.004
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        # axis -Y: positive q lifts the front (+X) edge up and back about the
        # rear hinge line (right-hand rule about -Y rotates +X toward +Z).
        axis=(0.0, -1.0, 0.0),
        # closed = 0 (lid flat); open lifts front edge up to ~105 deg back.
        motion_limits=MotionLimits(
            effort=60.0, velocity=2.0, lower=0.0, upper=math.radians(105.0)
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
    lid_joint = object_model.get_articulation("body_to_lid")

    # --- body and overall dumpster dimensions --------------------------------
    baabb = ctx.part_world_aabb(body)
    bext = _ext(baabb)
    ctx.check(
        "dumpster is ~1.8 m wide (walls + rim overhang)",
        BODY_W - 0.02 < bext[1] < BODY_W + 0.10,
        details=f"width={bext[1]}",
    )
    ctx.check(
        "dumpster is ~1.0-1.2 m deep and ~1.2-1.4 m tall",
        1.0 < bext[0] < 1.3 and 1.1 < bext[2] < 1.45,
        details=f"depth={bext[0]}, height={bext[2]}",
    )
    # front-load taper: top wider in X than the floor
    ctx.check(
        "body has front-load outward taper (top wider in X than base)",
        BODY_D_TOP > BODY_D_BOT + 0.10,
        details=f"top_d={BODY_D_TOP}, bot_d={BODY_D_BOT}",
    )

    # --- four caster wheels at the corners -----------------------------------
    corners = [
        (CASTER_HX, CASTER_HY),
        (CASTER_HX, -CASTER_HY),
        (-CASTER_HX, CASTER_HY),
        (-CASTER_HX, -CASTER_HY),
    ]
    for i, (cx, cy) in enumerate(corners):
        wname = f"wheel_{i}"
        jname = f"body_to_wheel_{i}"
        w = object_model.get_part(wname)
        j = object_model.get_articulation(jname)

        # joint is continuous
        ctx.check(
            f"{wname} joint is continuous",
            str(j.articulation_type).endswith("CONTINUOUS"),
            details=f"type={j.articulation_type}",
        )
        # joint axis is horizontal Y (wheel rolls like a cart wheel)
        ctx.check(
            f"{wname} rolls about horizontal Y axle",
            abs(j.axis[1]) > 0.99 and abs(j.axis[0]) < 0.01
            and abs(j.axis[2]) < 0.01,
            details=f"axis={j.axis}",
        )
        # joint origin at the axle (wheel center height, corner position)
        ctx.check(
            f"{wname} joint origin at axle center",
            abs(j.origin.xyz[2] - WHEEL_R) < 0.005,
            details=f"origin_z={j.origin.xyz[2]}, expected={WHEEL_R}",
        )

        # wheel bottom touches z ≈ 0 (ground)
        waabb = ctx.part_world_aabb(w)
        ctx.check(
            f"{wname} touches ground (min z ≈ 0)",
            waabb is not None and abs(waabb[0][2]) < 0.005,
            details=f"min_z={None if waabb is None else waabb[0][2]}",
        )

        # wheel is near its expected corner position
        wcenter = [(waabb[0][k] + waabb[1][k]) / 2.0 for k in range(3)]
        ctx.check(
            f"{wname} is at corner ({cx:+.2f}, {cy:+.2f})",
            abs(wcenter[0] - cx) < 0.02 and abs(wcenter[1] - cy) < 0.02,
            details=f"center=({wcenter[0]:.3f}, {wcenter[1]:.3f})",
        )

        # caster fork visual exists on body
        fork_vis = body.get_visual(f"caster_fork_{i}")
        ctx.check(
            f"caster_fork_{i} visual present on body",
            fork_vis is not None,
            details=f"visual={'found' if fork_vis else 'missing'}",
        )

    # --- each wheel fits between its fork legs (Y containment) --------------
    for i, (cx, cy) in enumerate(corners):
        w = object_model.get_part(f"wheel_{i}")
        ctx.expect_within(
            w, body,
            axes="y",
            inner_elem="wheel_tire",
            outer_elem=f"caster_fork_{i}",
            margin=0.001,
            name=f"wheel_{i} fits between fork legs (Y)",
        )

    # --- no fixed feet remain (replaced by casters) --------------------------
    # Body min z should be the caster fork bottom, not a foot block.
    ctx.check(
        "body has no fixed foot blocks (min z from caster forks)",
        baabb is not None and baabb[0][2] > 0.01,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )

    # --- slatted lid covers the mouth, sits on the rim (closed) --------------
    laabb = ctx.part_world_aabb(lid)
    lext = _ext(laabb)
    ctx.check(
        "lid spans the full mouth width",
        lext[1] > BODY_W - 0.01,
        details=f"lid_width={lext[1]}",
    )
    ctx.check(
        "closed lid lies flat on top of the body rim",
        abs(laabb[0][2] - baabb[1][2]) < 0.05 and lext[2] < 0.12,
        details=f"lid_min_z={laabb[0][2]}, body_top_z={baabb[1][2]}, lid_thk={lext[2]}",
    )
    # lid overhangs/rests on the rim -> intentional local seating overlap
    ctx.allow_overlap(
        body, lid,
        reason="The slatted lid rests on and overhangs the body rim along its rear hinge edge.",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.8,
        name="closed lid covers the body mouth footprint",
    )

    # --- primary articulation: rear-hinged flip lid about Y ------------------
    ctx.check(
        "lid joint is revolute",
        str(lid_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={lid_joint.articulation_type}",
    )
    ctx.check(
        "lid hinge axis is the rear top edge (local Y)",
        abs(lid_joint.axis[1]) > 0.99 and abs(lid_joint.axis[0]) < 0.01
        and abs(lid_joint.axis[2]) < 0.01,
        details=f"axis={lid_joint.axis}",
    )
    # hinge is at the REAR (-X) top edge
    ctx.check(
        "lid hinge is at the rear (-X) top edge of the body",
        lid_joint.origin.xyz[0] < -BODY_D_TOP / 2.0 + 0.05
        and lid_joint.origin.xyz[2] > WALL_TOP_Z - 0.05,
        details=f"hinge={lid_joint.origin.xyz}",
    )
    lim = lid_joint.motion_limits
    ctx.check(
        "lid open limit is realistic (~90-110 deg)",
        lim is not None and lim.upper is not None
        and math.radians(85) <= lim.upper <= math.radians(115),
        details=f"upper_rad={lim.upper}",
    )

    # opening the lid lifts the FRONT edge up and back (rear-hinged flip).
    rest_front_x = ctx.part_world_aabb(lid)[1][0]
    rest_top_z = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: math.radians(100)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top_z = open_aabb[1][2]
        open_front_x = open_aabb[1][0]
    ctx.check(
        "opening the lid swings the front edge up well above the rim",
        open_top_z > rest_top_z + 0.6,
        details=f"rest_top_z={rest_top_z}, open_top_z={open_top_z}",
    )
    ctx.check(
        "opening the lid pulls the leading edge rearward (rear-hinged)",
        open_front_x < rest_front_x - 0.3,
        details=f"rest_front_x={rest_front_x}, open_front_x={open_front_x}",
    )

    # --- the slats read as one connected lid (no floating slats) -------------
    lid_slats = lid.get_visual("lid_slats")
    ctx.check("lid slats visual present", lid_slats is not None, "expected lid_slats")

    return ctx.report()


object_model = build_object_model()
