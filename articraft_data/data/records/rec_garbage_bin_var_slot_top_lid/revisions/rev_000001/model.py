from __future__ import annotations

# Green steel commercial front-load dumpster fitted as a recycling drop bin.
#
# Real object: a weathered olive-green steel front-load waste container. The
# body is a tapered rectangular bin (wider at the top than the floor) with
# vertical corrugation ribs pressed into all four walls and a rolled top rim.
# The bin is hollow inside. A FIXED solid steel deck caps the top with a
# rectangular throw-in SLOT opening in the center. A small spring-return
# slatted FLAP door closes the slot: hinged at the REAR edge of the slot, it
# swings inward and back to let refuse drop, then falls shut.
#
# At the base are short steel feet and two front forklift/skid pockets; on
# each side is a lifting trunnion pocket used by the truck arms.
#
# Coordinate convention:
#   - +Z is up; the base feet sit at z = 0.
#   - The truck approaches from the front (+X).
#   - Centerline is y = 0; the bin is left/right symmetric across it.
#
# Root structure: the body is the root. The fixed deck is inlined as a body
# visual. The slatted flap is the sole articulated child, hinged at the rear
# edge of the slot (REVOLUTE, axis along Y).
#
# Articulation:
#   - body_to_flap : REVOLUTE, rear-hinged slot flap, axis along Y.

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

# Fixed steel deck with rectangular slot
DECK_T = 0.018            # deck plate thickness
SLOT_W = 0.55             # slot width (Y)
SLOT_D = 0.40             # slot depth (X)

# Flap door (slatted, fits in the slot)
FLAP_SLATS = 7            # number of front-to-back slats across the flap
FLAP_GAP = 0.010          # gap between slats
FLAP_THK = 0.014          # slat thickness
FLAP_CLEARANCE = 0.012    # clearance around the flap in the slot
FLAP_W = SLOT_W - FLAP_CLEARANCE   # flap width
FLAP_D = SLOT_D - FLAP_CLEARANCE   # flap depth

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


def _deck_mesh() -> MeshGeometry:
    """Fixed steel deck plate with a centered rectangular slot opening.

    The deck sits on top of the body rim at WALL_TOP_Z. Four deck panels
    surround the slot: front strip, rear strip, and two side strips. A thin
    downward slot lip gives the opening a formed-edge look.
    """
    geo = MeshGeometry()
    hd = BODY_D_TOP / 2.0   # body half-depth at top
    hw = BODY_W / 2.0       # body half-width
    shd = SLOT_D / 2.0      # slot half-depth (X)
    shw = SLOT_W / 2.0      # slot half-width (Y)
    zc = WALL_TOP_Z + DECK_T / 2.0  # deck plate center Z

    # --- four deck panels around the slot ---

    # Rear strip: X from -hd to -shd, full Y width
    rear_len = hd - shd
    rear = BoxGeometry((rear_len, BODY_W, DECK_T))
    rear.translate(-(hd + shd) / 2.0, 0.0, zc)
    geo.merge(rear)

    # Front strip: X from +shd to +hd, full Y width
    front = BoxGeometry((rear_len, BODY_W, DECK_T))
    front.translate((hd + shd) / 2.0, 0.0, zc)
    geo.merge(front)

    # Left strip: X from -shd to +shd, Y from -hw to -shw
    side_y_len = hw - shw
    left = BoxGeometry((SLOT_D, side_y_len, DECK_T))
    left.translate(0.0, -(hw + shw) / 2.0, zc)
    geo.merge(left)

    # Right strip: X from -shd to +shd, Y from +shw to +hw
    right = BoxGeometry((SLOT_D, side_y_len, DECK_T))
    right.translate(0.0, (hw + shw) / 2.0, zc)
    geo.merge(right)

    # --- slot lip: thin downward flange around the slot opening ---
    lip_h = 0.030          # lip height below deck bottom
    lip_t = 0.006          # lip thickness
    z_lip_center = WALL_TOP_Z - lip_h / 2.0

    # Rear lip (at X = -shd, inside edge)
    rear_lip = BoxGeometry((lip_t, SLOT_W, lip_h))
    rear_lip.translate(-shd + lip_t / 2.0, 0.0, z_lip_center)
    geo.merge(rear_lip)

    # Front lip (at X = +shd)
    front_lip = BoxGeometry((lip_t, SLOT_W, lip_h))
    front_lip.translate(shd - lip_t / 2.0, 0.0, z_lip_center)
    geo.merge(front_lip)

    # Left lip (at Y = -shw)
    left_lip = BoxGeometry((SLOT_D - 2 * lip_t, lip_t, lip_h))
    left_lip.translate(0.0, -shw + lip_t / 2.0, z_lip_center)
    geo.merge(left_lip)

    # Right lip (at Y = +shw)
    right_lip = BoxGeometry((SLOT_D - 2 * lip_t, lip_t, lip_h))
    right_lip.translate(0.0, shw - lip_t / 2.0, z_lip_center)
    geo.merge(right_lip)

    return geo


def _make_flap_slat(depth: float, width: float, thickness: float) -> BoxGeometry:
    """Shared slat geometry helper: a thin rectangular bar for the flap."""
    return BoxGeometry((depth, width, thickness))


def _flap_frame_mesh() -> MeshGeometry:
    """Connecting frame for the flap: a hinge strip and front cross-bar.

    Authored in the flap's local frame (hinge at origin, flap extends in +X).
    The frame ties all slats together so the flap reads as one connected part.
    The slats overlap with the hinge strip and front bar for connectivity.
    """
    geo = MeshGeometry()
    bar_w = 0.022   # cross-bar width in X
    frame_h = FLAP_THK + 0.004  # slightly taller than slats for embed

    # Hinge strip at the rear (x ≈ 0), spanning full flap width
    hinge = BoxGeometry((bar_w, FLAP_W, frame_h))
    hinge.translate(bar_w / 2.0, 0.0, -frame_h / 2.0)
    geo.merge(hinge)

    # Front cross-bar at the leading edge
    front = BoxGeometry((bar_w, FLAP_W, frame_h))
    front.translate(FLAP_D - bar_w / 2.0, 0.0, -frame_h / 2.0)
    geo.merge(front)

    # Mid-span cross-bar for extra rigidity
    mid = BoxGeometry((bar_w * 0.8, FLAP_W, frame_h * 0.8))
    mid.translate(FLAP_D / 2.0, 0.0, -frame_h / 2.0)
    geo.merge(mid)

    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dumpster")

    green = model.material("steel_green", rgba=(0.27, 0.40, 0.20, 1.0))
    green_dark = model.material("steel_green_dark", rgba=(0.20, 0.31, 0.15, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.22, 0.24, 0.20, 1.0))
    flap_steel = model.material("flap_steel", rgba=(0.18, 0.20, 0.16, 1.0))
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
    # Fixed steel deck with slot opening (inlined as body visual)
    body.visual(mesh_from_geometry(_deck_mesh(), "deck_plate"),
                material=deck_steel, name="deck_plate")
    body.inertial = Inertial.from_geometry(
        Box((BODY_D_TOP, BODY_W, BODY_H)),
        mass=130.0,
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + BODY_H * 0.45)),
    )

    # ---- slatted flap door (hinged at rear edge of slot) --------------------
    flap = model.part("flap")
    # Connecting frame (hinge strip + cross-bars)
    flap.visual(mesh_from_geometry(_flap_frame_mesh(), "flap_frame"),
                material=flap_steel, name="flap_frame")
    # Individual front-to-back slats emitted via loop with shared helper
    slat_pitch = (FLAP_W - FLAP_GAP) / FLAP_SLATS
    slat_w = slat_pitch - FLAP_GAP
    for i in range(FLAP_SLATS):
        yc = -FLAP_W / 2.0 + slat_pitch * (i + 0.5)
        slat_geo = _make_flap_slat(FLAP_D, slat_w, FLAP_THK)
        flap.visual(
            mesh_from_geometry(slat_geo, f"flap_slat_{i}"),
            origin=Origin(xyz=(FLAP_D / 2.0, yc, -FLAP_THK / 2.0)),
            material=flap_steel,
            name=f"flap_slat_{i}",
        )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_D, FLAP_W, FLAP_THK)),
        mass=3.5,
        origin=Origin(xyz=(FLAP_D / 2.0, 0.0, -FLAP_THK / 2.0)),
    )

    # Hinge at the REAR (-X) edge of the slot, at the deck top surface.
    # The flap local frame has its hinge at origin, and the flap panel extends
    # forward (+X). axis = +Y: positive q swings the front (+X) edge downward
    # (-Z) into the bin (right-hand rule about +Y rotates +X toward -Z).
    hinge_x = -SLOT_D / 2.0
    hinge_z = WALL_TOP_Z + DECK_T
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        # closed = 0 (flap flat in slot); open = positive angle swings inward
        # down to about 85 degrees (flap nearly vertical, dropping refuse in).
        motion_limits=MotionLimits(
            effort=15.0, velocity=4.0, lower=0.0, upper=math.radians(85.0)
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("body_to_flap")

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
        1.0 < bext[0] < 1.3 and 1.1 < bext[2] < 1.4,
        details=f"depth={bext[0]}, height={bext[2]}",
    )
    # front-load taper: top wider in X than the floor
    ctx.check(
        "body has front-load outward taper (top wider in X than base)",
        BODY_D_TOP > BODY_D_BOT + 0.10,
        details=f"top_d={BODY_D_TOP}, bot_d={BODY_D_BOT}",
    )

    # --- fixed deck with slot is present on the body -------------------------
    deck_vis = body.get_visual("deck_plate")
    ctx.check("deck plate visual present on body", deck_vis is not None,
              "expected deck_plate visual")

    # --- flap fits in the slot, sits near the deck level (closed) -----------
    faabb = ctx.part_world_aabb(flap)
    fext = _ext(faabb)
    ctx.check(
        "flap spans roughly the slot width",
        fext[1] > FLAP_W - 0.02,
        details=f"flap_width={fext[1]}",
    )
    ctx.check(
        "closed flap sits near the deck top level",
        abs(faabb[1][2] - (WALL_TOP_Z + DECK_T)) < 0.03,
        details=f"flap_top_z={faabb[1][2]}, deck_top_z={WALL_TOP_Z + DECK_T}",
    )
    # Flap footprint is within the slot (within the deck opening)
    ctx.expect_within(
        flap, body, axes="xy", margin=0.05,
        name="closed flap fits within the body footprint",
    )

    # --- primary articulation: rear-hinged flap about Y ---------------------
    ctx.check(
        "flap joint is revolute",
        str(flap_joint.articulation_type).endswith("REVOLUTE"),
        details=f"type={flap_joint.articulation_type}",
    )
    ctx.check(
        "flap hinge axis is +Y (rear edge of slot, positive swings inward)",
        flap_joint.axis[1] > 0.99 and abs(flap_joint.axis[0]) < 0.01
        and abs(flap_joint.axis[2]) < 0.01,
        details=f"axis={flap_joint.axis}",
    )
    # Hinge is at the rear (-X) edge of the slot, at deck level
    ctx.check(
        "flap hinge is at the rear (-X) edge of the slot at deck height",
        abs(flap_joint.origin.xyz[0] - (-SLOT_D / 2.0)) < 0.01
        and abs(flap_joint.origin.xyz[2] - (WALL_TOP_Z + DECK_T)) < 0.01,
        details=f"hinge_origin={flap_joint.origin.xyz}",
    )
    lim = flap_joint.motion_limits
    ctx.check(
        "flap open limit is realistic (~70-90 deg inward swing)",
        lim is not None and lim.upper is not None
        and math.radians(65) <= lim.upper <= math.radians(95),
        details=f"upper_rad={lim.upper}",
    )
    ctx.check(
        "flap closed limit is zero",
        lim is not None and lim.lower is not None and abs(lim.lower) < 0.01,
        details=f"lower_rad={lim.lower}",
    )

    # Opening the flap swings the front edge DOWN into the bin.
    rest_front_x = ctx.part_world_aabb(flap)[1][0]
    rest_bottom_z = ctx.part_world_aabb(flap)[0][2]
    with ctx.pose({flap_joint: math.radians(80)}):
        open_aabb = ctx.part_world_aabb(flap)
        open_bottom_z = open_aabb[0][2]
        open_front_x = open_aabb[1][0]
    ctx.check(
        "opening the flap swings the front edge down into the bin",
        open_bottom_z < rest_bottom_z - 0.15,
        details=f"rest_bottom_z={rest_bottom_z}, open_bottom_z={open_bottom_z}",
    )
    ctx.check(
        "opening the flap pulls the front edge rearward (hinged at rear)",
        open_front_x < rest_front_x - 0.10,
        details=f"rest_front_x={rest_front_x}, open_front_x={open_front_x}",
    )

    # --- flap has slatted construction (individual slats present) ------------
    slat_names = [f"flap_slat_{i}" for i in range(FLAP_SLATS)]
    for sname in slat_names:
        sv = flap.get_visual(sname)
        ctx.check(f"flap slat {sname} present", sv is not None,
                  f"expected {sname} visual on flap")

    # Frame visual connects all slats
    frame_vis = flap.get_visual("flap_frame")
    ctx.check("flap frame visual present", frame_vis is not None,
              "expected flap_frame visual")

    # --- hinge contact: flap frame contacts deck at the slot rear edge -------
    # The flap hinge strip sits right at the deck edge — intentional hinge embed
    ctx.allow_overlap(
        body, flap,
        elem_a="deck_plate",
        elem_b="flap_frame",
        reason="The flap hinge strip embeds slightly into the deck rear edge "
               "at the hinge pivot, representing the hinge pin seating.",
    )
    ctx.expect_contact(
        flap, body,
        elem_a="flap_frame", elem_b="deck_plate",
        contact_tol=0.005,
        name="flap frame contacts deck at hinge edge",
    )

    return ctx.report()


object_model = build_object_model()
