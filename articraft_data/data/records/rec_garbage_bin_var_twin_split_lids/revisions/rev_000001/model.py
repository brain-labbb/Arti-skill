from __future__ import annotations

# Green steel commercial front-load dumpster with TWO independent split half-lids.
#
# Real object: a weathered olive-green steel front-load waste container. The
# body is a tapered rectangular bin (a little wider at the top than the floor)
# with vertical corrugation ribs pressed into all four walls and a rolled top
# rim. The bin is open inside (hollow). Instead of one full-width lid, the
# mouth is capped by TWO INDEPENDENT SLATTED STEEL HALF-LIDS that meet at the
# centerline (y=0). Each half-lid is hinged along the REAR top edge on its own
# revolute hinge and flips up and back independently, so one side can be opened
# while the other stays shut. At the base are short steel feet and two front
# forklift/skid pockets; on each side is a lifting trunnion pocket used by the
# truck arms.
#
# Coordinate convention:
#   - +Z is up; the base feet sit at z = 0.
#   - The truck approaches from the front (+X). Each lid hinges along the REAR
#     (-X) top edge and lifts up/back.
#   - Centerline is y = 0; the bin is left/right symmetric across it.
#
# Root structure: the body is the root. Each half-lid is hinged at the rear
# top edge (REVOLUTE, axis along Y). Feet, forklift pockets, side trunnions,
# and rim are fixed body detail.
#
# Articulations:
#   - body_to_lid_0 : REVOLUTE, rear-hinged left (-Y) half-lid
#   - body_to_lid_1 : REVOLUTE, rear-hinged right (+Y) half-lid

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
# A ~2 cubic yard front-load dumpster.
BODY_W = 1.80              # width (Y)
BODY_D_BOT = 1.00         # depth at the floor (X)
BODY_D_TOP = 1.18         # depth at the top (X) -> outward taper toward the top
BODY_H = 1.15             # body wall height (top rim z above the feet)
WALL_T = 0.012
FLOOR_T = 0.020

FOOT_H = 0.07             # height of the base feet/skids
FOOT_SIZE = 0.12

RIM_T = 0.030             # rolled top rim thickness
RIB_COUNT_SIDE = 9        # vertical corrugation ribs along the long (Y) walls
RIB_COUNT_END = 5         # vertical ribs on the short (X) end walls
RIB_W = 0.045
RIB_PROUD = 0.020         # how far ribs stand proud of the wall
RIB_EMBED = 0.004         # slight embed prevents coplanar z-fighting

LID_SLATS = 11            # number of slats across the FULL lid (parent reference)
LID_GAP = 0.012           # gap between slats
LID_THK = 0.030
LID_OVER = 0.020          # lid overhang past the rim
CENTER_GAP = 0.012        # gap between the two half-lids at the centerline

WALL_TOP_Z = FOOT_H + BODY_H   # top rim z


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


def _feet_and_pockets() -> MeshGeometry:
    """Base feet/skids and two front forklift pockets + side trunnions."""
    geo = MeshGeometry()
    hd = BODY_D_BOT / 2.0 - FOOT_SIZE / 2.0 - 0.02
    hw = BODY_W / 2.0 - FOOT_SIZE / 2.0 - 0.02
    # four corner feet rising from z=0 to FOOT_H
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            foot = BoxGeometry((FOOT_SIZE, FOOT_SIZE, FOOT_H))
            foot.translate(sx * hd, sy * hw, FOOT_H / 2.0)
            geo.merge(foot)
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


def _half_lid_mesh() -> MeshGeometry:
    """Slatted steel half-lid: parallel ribs running front-to-back (X), with gaps.

    Authored in the half-lid's OWN frame with the hinge axis (Y) at local x=0,
    z=0 along the REAR edge. The lid panel extends forward (+X) from the hinge
    and the slat ribs span the half-width. The mesh is centered at local y=0,
    spanning ±half_active_w/2. A thin base skin ties all slats together so it
    reads as one connected half-lid, not floating slats.

    The same mesh helper is called for both half-lids (i=0 and i=1); the Y
    placement offset is applied by the articulation origin.
    """
    geo = MeshGeometry()
    depth = BODY_D_TOP + 2 * LID_OVER       # front-to-back length of the lid
    full_width = BODY_W + 2 * LID_OVER       # total width across the full mouth
    half_active_w = full_width / 2.0 - CENTER_GAP / 2.0  # active width per half

    # Use the same slat pitch as the parent full-width lid
    slat_pitch = (full_width - LID_GAP) / LID_SLATS
    slat_w = slat_pitch - LID_GAP

    # Number of slats that fit in the half-width
    n_half = max(1, int(math.floor(half_active_w / slat_pitch)))

    # base skin tying the slats together
    skin = BoxGeometry((depth, half_active_w, 0.008))
    skin.translate(depth / 2.0, 0.0, -0.004)
    geo.merge(skin)

    # Distribute slats centered in the half-width
    group_span = n_half * slat_pitch
    start_y = -group_span / 2.0
    for i in range(n_half):
        yc = start_y + slat_pitch * (i + 0.5)
        slat = BoxGeometry((depth, slat_w, LID_THK))
        slat.translate(depth / 2.0, yc, LID_THK / 2.0)
        geo.merge(slat)

    # a small front lift handle/lip on the leading (+X) edge
    handle = BoxGeometry((0.05, half_active_w * 0.4, 0.05))
    handle.translate(depth - 0.025, 0.0, LID_THK + 0.005)
    geo.merge(handle)

    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dumpster")

    green = model.material("steel_green", rgba=(0.27, 0.40, 0.20, 1.0))
    green_dark = model.material("steel_green_dark", rgba=(0.20, 0.31, 0.15, 1.0))
    lid_dark = model.material("lid_steel", rgba=(0.16, 0.18, 0.15, 1.0))
    lid_dark_alt = model.material("lid_steel_alt", rgba=(0.14, 0.17, 0.13, 1.0))
    foot_dark = model.material("foot_steel", rgba=(0.14, 0.15, 0.14, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(mesh_from_geometry(_tapered_body(), "body_shell"),
                material=green, name="body_shell")
    body.visual(mesh_from_geometry(_ribs(), "body_ribs"),
                material=green_dark, name="body_ribs")
    body.visual(mesh_from_geometry(_rim(), "top_rim"),
                material=green_dark, name="top_rim")
    body.visual(mesh_from_geometry(_feet_and_pockets(), "base_hardware"),
                material=foot_dark, name="base_hardware")
    body.inertial = Inertial.from_geometry(
        Box((BODY_D_TOP, BODY_W, BODY_H)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + BODY_H * 0.45)),
    )

    # ---- split slatted rear-hinged half-lids --------------------------------
    # Two independent half-lids meeting at the centerline (y=0), each hinged
    # along the rear top edge on its own revolute hinge. Generated via a
    # for-i-in-range(2) loop over a shared half-lid mesh helper, mirrored
    # across y=0 with name_i style naming and a uniform revolute hinge policy.
    full_width = BODY_W + 2 * LID_OVER
    hinge_x = -BODY_D_TOP / 2.0 - LID_OVER
    hinge_z = WALL_TOP_Z + 0.004
    lid_materials = [lid_dark, lid_dark_alt]

    for i in range(2):
        side_sign = -1.0 if i == 0 else 1.0
        y_off = side_sign * (full_width / 4.0)

        lid_i = model.part(f"lid_{i}")
        lid_i.visual(
            mesh_from_geometry(_half_lid_mesh(), f"lid_{i}_slats"),
            material=lid_materials[i],
            name=f"lid_{i}_slats",
        )
        lid_i.inertial = Inertial.from_geometry(
            Box((BODY_D_TOP, full_width / 2.0, LID_THK)),
            mass=7.0,
            origin=Origin(xyz=(BODY_D_TOP / 2.0, 0.0, 0.0)),
        )

        # Hinge at the REAR top edge, offset in Y to the center of this half.
        # The half-lid local frame has its hinge at x=0 and the lid extends to
        # +X; placed at the rear (-X) rim so closed it lies flat over its half
        # of the mouth. axis = -Y: positive q lifts the front (+X) edge up and
        # back (right-hand rule about -Y rotates +X toward +Z).
        model.articulation(
            f"body_to_lid_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=lid_i,
            origin=Origin(xyz=(hinge_x, y_off, hinge_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=math.radians(105.0)
            ),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    joint_0 = object_model.get_articulation("body_to_lid_0")
    joint_1 = object_model.get_articulation("body_to_lid_1")

    # --- base feet sit at z ~ 0; overall dumpster size realistic -------------
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "base feet sit at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.005,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    bext = _ext(baabb)
    ctx.check(
        "dumpster is ~1.8 m wide (walls + rim overhang)",
        BODY_W - 0.02 < bext[1] < BODY_W + 0.10,
        details=f"width={bext[1]}",
    )
    ctx.check(
        "dumpster is ~1.0-1.2 m deep and ~1.2 m tall",
        1.0 < bext[0] < 1.3 and 1.1 < bext[2] < 1.3,
        details=f"depth={bext[0]}, height={bext[2]}",
    )
    ctx.check(
        "body has front-load outward taper (top wider in X than base)",
        BODY_D_TOP > BODY_D_BOT + 0.10,
        details=f"top_d={BODY_D_TOP}, bot_d={BODY_D_BOT}",
    )

    # --- two independent half-lids exist and cover the mouth -----------------
    full_width = BODY_W + 2 * LID_OVER

    for i, (lid_i, joint_i) in enumerate([(lid_0, joint_0), (lid_1, joint_1)]):
        laabb = ctx.part_world_aabb(lid_i)
        lext = _ext(laabb)

        # Each half-lid spans roughly half the mouth width
        ctx.check(
            f"lid_{i} spans roughly half the mouth width",
            lext[1] > full_width / 2.0 - 0.05,
            details=f"lid_{i}_width={lext[1]}, expected~{full_width / 2.0}",
        )

        # Closed half-lid lies flat on top of the body rim
        ctx.check(
            f"closed lid_{i} lies flat on top of the body rim",
            abs(laabb[0][2] - baabb[1][2]) < 0.05 and lext[2] < 0.12,
            details=f"lid_{i}_min_z={laabb[0][2]}, body_top_z={baabb[1][2]}, thk={lext[2]}",
        )

        # Each half-lid rests on the rim (intentional seating overlap)
        ctx.allow_overlap(
            body, lid_i,
            reason=f"Half-lid lid_{i} rests on and overhangs the body rim along its rear hinge edge.",
        )

        # Each half-lid covers its half of the mouth footprint
        ctx.expect_overlap(
            lid_i, body, axes="xy", min_overlap=0.35,
            name=f"closed lid_{i} covers its half of the body mouth footprint",
        )

        # Joint is revolute with correct axis
        ctx.check(
            f"lid_{i} joint is revolute",
            str(joint_i.articulation_type).endswith("REVOLUTE"),
            details=f"type={joint_i.articulation_type}",
        )
        ctx.check(
            f"lid_{i} hinge axis is along Y (rear top edge)",
            abs(joint_i.axis[1]) > 0.99 and abs(joint_i.axis[0]) < 0.01
            and abs(joint_i.axis[2]) < 0.01,
            details=f"axis={joint_i.axis}",
        )

        # Hinge is at the REAR (-X) top edge
        ctx.check(
            f"lid_{i} hinge is at the rear (-X) top edge of the body",
            joint_i.origin.xyz[0] < -BODY_D_TOP / 2.0 + 0.05
            and joint_i.origin.xyz[2] > WALL_TOP_Z - 0.05,
            details=f"hinge={joint_i.origin.xyz}",
        )

        # Open limit is realistic
        lim = joint_i.motion_limits
        ctx.check(
            f"lid_{i} open limit is realistic (~90-110 deg)",
            lim is not None and lim.upper is not None
            and math.radians(85) <= lim.upper <= math.radians(115),
            details=f"upper_rad={lim.upper}",
        )

        # Slats visual present
        slats_vis = lid_i.get_visual(f"lid_{i}_slats")
        ctx.check(
            f"lid_{i} slats visual present",
            slats_vis is not None,
            details=f"expected lid_{i}_slats",
        )

    # --- the two half-lids are on opposite sides of the centerline -----------
    ctx.check(
        "lid_0 hinge is on the -Y side of centerline",
        joint_0.origin.xyz[1] < -0.1,
        details=f"lid_0_hinge_y={joint_0.origin.xyz[1]}",
    )
    ctx.check(
        "lid_1 hinge is on the +Y side of centerline",
        joint_1.origin.xyz[1] > 0.1,
        details=f"lid_1_hinge_y={joint_1.origin.xyz[1]}",
    )

    # --- independent opening: lid_0 opens while lid_1 stays closed ----------
    rest_0_top_z = ctx.part_world_aabb(lid_0)[1][2]
    rest_0_front_x = ctx.part_world_aabb(lid_0)[1][0]
    rest_1_top_z = ctx.part_world_aabb(lid_1)[1][2]

    with ctx.pose({joint_0: math.radians(100)}):
        open_0_aabb = ctx.part_world_aabb(lid_0)
        open_0_top_z = open_0_aabb[1][2]
        open_0_front_x = open_0_aabb[1][0]
        # lid_1 should remain closed (not affected by lid_0 opening)
        closed_1_top_z = ctx.part_world_aabb(lid_1)[1][2]

    ctx.check(
        "opening lid_0 swings its front edge up well above the rim",
        open_0_top_z > rest_0_top_z + 0.5,
        details=f"rest_top_z={rest_0_top_z}, open_top_z={open_0_top_z}",
    )
    ctx.check(
        "opening lid_0 pulls its leading edge rearward (rear-hinged)",
        open_0_front_x < rest_0_front_x - 0.2,
        details=f"rest_front_x={rest_0_front_x}, open_front_x={open_0_front_x}",
    )
    ctx.check(
        "lid_1 stays closed when lid_0 opens (independent operation)",
        abs(closed_1_top_z - rest_1_top_z) < 0.02,
        details=f"rest_1_top_z={rest_1_top_z}, during_open_1_top_z={closed_1_top_z}",
    )

    # --- independent opening: lid_1 opens while lid_0 stays closed ----------
    rest_1_front_x = ctx.part_world_aabb(lid_1)[1][0]

    with ctx.pose({joint_1: math.radians(100)}):
        open_1_aabb = ctx.part_world_aabb(lid_1)
        open_1_top_z = open_1_aabb[1][2]
        open_1_front_x = open_1_aabb[1][0]
        closed_0_top_z = ctx.part_world_aabb(lid_0)[1][2]

    ctx.check(
        "opening lid_1 swings its front edge up well above the rim",
        open_1_top_z > rest_1_top_z + 0.5,
        details=f"rest_top_z={rest_1_top_z}, open_top_z={open_1_top_z}",
    )
    ctx.check(
        "opening lid_1 pulls its leading edge rearward (rear-hinged)",
        open_1_front_x < rest_1_front_x - 0.2,
        details=f"rest_front_x={rest_1_front_x}, open_front_x={open_1_front_x}",
    )
    ctx.check(
        "lid_0 stays closed when lid_1 opens (independent operation)",
        abs(closed_0_top_z - rest_0_top_z) < 0.02,
        details=f"rest_0_top_z={rest_0_top_z}, during_open_0_top_z={closed_0_top_z}",
    )

    # --- both can open simultaneously ----------------------------------------
    with ctx.pose({joint_0: math.radians(90), joint_1: math.radians(90)}):
        both_0_top = ctx.part_world_aabb(lid_0)[1][2]
        both_1_top = ctx.part_world_aabb(lid_1)[1][2]
    ctx.check(
        "both half-lids can open simultaneously",
        both_0_top > rest_0_top_z + 0.4 and both_1_top > rest_1_top_z + 0.4,
        details=f"lid_0_top={both_0_top}, lid_1_top={both_1_top}",
    )

    return ctx.report()


object_model = build_object_model()
