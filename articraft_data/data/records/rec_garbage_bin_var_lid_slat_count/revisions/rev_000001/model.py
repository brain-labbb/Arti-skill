from __future__ import annotations

# Green steel commercial front-load dumpster with a rear-hinged slatted lid.
#
# Real object: a weathered olive-green steel front-load waste container. The
# body is a tapered rectangular bin (a little wider at the top than the floor)
# with vertical corrugation ribs pressed into all four walls and a rolled top
# rim. The bin is open inside (hollow). A slatted steel LID caps the top: a set
# of parallel ribs/slats running front-to-back, hinged along the REAR top edge
# so it flips up and back to open. At the base are short steel feet and two
# front forklift/skid pockets; on each side is a lifting trunnion pocket used by
# the truck arms.
#
# Coordinate convention:
#   - +Z is up; the base feet sit at z = 0.
#   - The truck approaches from the front (+X). The lid hinges along the REAR
#     (-X) top edge and lifts up/back.
#   - Centerline is y = 0; the bin is left/right symmetric across it.
#
# Root structure: the body is the root. The slatted lid is hinged at the rear
# top edge (REVOLUTE, axis along Y). Feet, forklift pockets, side trunnions, and
# rim are fixed body detail.
#
# Articulation:
#   - body_to_lid : REVOLUTE, rear-hinged flip lid, axis along Y (rear top edge).

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

LID_SLAT_COUNT = 11       # PARAMETRIC: single multiplicity parameter for slat count
LID_GAP = 0.012           # gap between slats
LID_THK = 0.030
LID_OVER = 0.020          # lid overhang past the rim
LID_SKIN_THK = 0.008      # thin connecting skin thickness

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


def _slat_geometry(depth: float, slat_w: float) -> MeshGeometry:
    """Single lid slat bar centered at the origin.

    Shared geometry helper used by the parametric slat loop. Each slat is a
    rectangular bar running front-to-back (X) with width `slat_w` (Y) and
    thickness `LID_THK` (Z).
    """
    geo = MeshGeometry()
    geo.merge(BoxGeometry((depth, slat_w, LID_THK)))
    return geo


def _lid_skin_geometry(depth: float, width: float) -> MeshGeometry:
    """Thin connecting skin that ties all slats into one connected lid panel."""
    geo = MeshGeometry()
    geo.merge(BoxGeometry((depth, width, LID_SKIN_THK)))
    return geo


def _handle_geometry(width: float) -> MeshGeometry:
    """Front lift handle/lip on the leading edge of the lid."""
    geo = MeshGeometry()
    geo.merge(BoxGeometry((0.05, width * 0.4, 0.05)))
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dumpster")

    green = model.material("steel_green", rgba=(0.27, 0.40, 0.20, 1.0))
    green_dark = model.material("steel_green_dark", rgba=(0.20, 0.31, 0.15, 1.0))
    lid_dark = model.material("lid_steel", rgba=(0.16, 0.18, 0.15, 1.0))
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

    # ---- slatted rear-hinged lid -------------------------------------------
    # Parametric slat array: one multiplicity parameter (LID_SLAT_COUNT) drives
    # the entire lid panel. A thin connecting skin ties all slats into one
    # connected lid assembly.
    lid = model.part("lid")

    # Compute lid dimensions and slat layout from the single parameter
    lid_depth = BODY_D_TOP + 2 * LID_OVER       # front-to-back length
    lid_width = BODY_W + 2 * LID_OVER           # across the slats (Y)
    slat_pitch = (lid_width - LID_GAP) / LID_SLAT_COUNT
    slat_w = slat_pitch - LID_GAP

    # Connecting skin: thin plate at the bottom that ties all slats together
    # Positioned so its top face is at z=0 (hinge plane), extending slightly below
    lid.visual(
        mesh_from_geometry(_lid_skin_geometry(lid_depth, lid_width), "lid_skin"),
        material=lid_dark,
        name="lid_skin",
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, -LID_SKIN_THK / 2.0)),
    )

    # Slats: parametric array driven by LID_SLAT_COUNT
    # Each slat sits on top of the skin with a small intentional embed for connectivity
    for i in range(LID_SLAT_COUNT):
        yc = -lid_width / 2.0 + slat_pitch * (i + 0.5)
        slat_name = f"slat_{i}"
        lid.visual(
            mesh_from_geometry(_slat_geometry(lid_depth, slat_w), slat_name),
            material=lid_dark,
            name=slat_name,
            origin=Origin(xyz=(lid_depth / 2.0, yc, LID_THK / 2.0)),
        )

    # Front lift handle/lip on the leading (+X) edge
    lid.visual(
        mesh_from_geometry(_handle_geometry(lid_width), "lid_handle"),
        material=lid_dark,
        name="lid_handle",
        origin=Origin(xyz=(lid_depth - 0.025, 0.0, LID_THK + 0.025)),
    )
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
    # front-load taper: top wider in X than the floor
    ctx.check(
        "body has front-load outward taper (top wider in X than base)",
        BODY_D_TOP > BODY_D_BOT + 0.10,
        details=f"top_d={BODY_D_TOP}, bot_d={BODY_D_BOT}",
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

    # --- the slats read as one connected lid (parametric slat array) ----------
    # Verify the parametric slat count drives the correct number of named visuals
    lid_skin = lid.get_visual("lid_skin")
    ctx.check("lid connecting skin present", lid_skin is not None,
              "expected lid_skin visual to tie slats together")

    slat_visuals = [lid.get_visual(f"slat_{i}") for i in range(LID_SLAT_COUNT)]
    all_slats_present = all(v is not None for v in slat_visuals)
    ctx.check(
        f"parametric slat array has {LID_SLAT_COUNT} slats (slat_0..slat_{LID_SLAT_COUNT - 1})",
        all_slats_present,
        details=f"missing slats among {LID_SLAT_COUNT} expected",
    )

    lid_handle = lid.get_visual("lid_handle")
    ctx.check("lid front handle present", lid_handle is not None,
              "expected lid_handle visual")

    # Verify slats span the full lid width at regular pitch
    if all_slats_present and LID_SLAT_COUNT >= 2:
        first_y = slat_visuals[0].origin.xyz[1] if slat_visuals[0].origin else None
        last_y = slat_visuals[-1].origin.xyz[1] if slat_visuals[-1].origin else None
        if first_y is not None and last_y is not None:
            slat_span = abs(last_y - first_y)
            test_lid_width = BODY_W + 2 * LID_OVER
            test_slat_pitch = (test_lid_width - LID_GAP) / LID_SLAT_COUNT
            expected_span = (LID_SLAT_COUNT - 1) * test_slat_pitch
            ctx.check(
                "slats are regularly spaced across the lid width",
                abs(slat_span - expected_span) < 0.01,
                details=f"slat_span={slat_span:.4f}, expected={expected_span:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
