from __future__ import annotations

# Green steel commercial front-load dumpster with a rear-hinged slatted lid,
# set up as a two-caster tilt-and-roll bin.
#
# Real object: a weathered olive-green steel front-load waste container. The
# body is a tapered rectangular bin (a little wider at the top than the floor)
# with vertical corrugation ribs pressed into all four walls and a rolled top
# rim. The bin is open inside (hollow). A slatted steel LID caps the top: a set
# of parallel ribs/slats running front-to-back, hinged along the REAR top edge
# so it flips up and back to open.
#
# Two-caster tilt-and-roll setup: the rear corners keep fixed steel feet while
# the two front corners each carry a mounting plate, a caster fork bracket, and
# a round rubber wheel on a horizontal axle. The bin tips back onto the two
# front wheels to be rolled.
#
# Coordinate convention:
#   - +Z is up; the base feet sit at z = 0.
#   - The truck approaches from the front (+X). The lid hinges along the REAR
#     (-X) top edge and lifts up/back.
#   - Centerline is y = 0; the bin is left/right symmetric across it.
#
# Root structure: the body is the root. The slatted lid is hinged at the rear
# top edge (REVOLUTE, axis along Y). Rear feet, forklift pockets, side
# trunnions, and rim are fixed body detail. Caster fork brackets are inline
# body visuals; each wheel is a separate part with a CONTINUOUS roll joint.
#
# Articulations:
#   - body_to_lid    : REVOLUTE, rear-hinged flip lid, axis along Y.
#   - body_to_wheel_0: CONTINUOUS, front-right caster wheel roll, axis along Y.
#   - body_to_wheel_1: CONTINUOUS, front-left caster wheel roll, axis along Y.

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
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
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

LID_SLATS = 11            # number of slats across the lid
LID_GAP = 0.012           # gap between slats
LID_THK = 0.030
LID_OVER = 0.020          # lid overhang past the rim

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
    """Rear feet/skids, two front forklift pockets, and side trunnions.

    The two front (+X) corners no longer have fixed feet; they carry caster
    assemblies instead (fork + plate as body visuals, wheel as a separate
    articulated part).
    """
    geo = MeshGeometry()
    hd = BODY_D_BOT / 2.0 - FOOT_SIZE / 2.0 - 0.02
    hw = BODY_W / 2.0 - FOOT_SIZE / 2.0 - 0.02
    # only the two REAR (-X) corner feet remain fixed
    for sy in (1.0, -1.0):
        foot = BoxGeometry((FOOT_SIZE, FOOT_SIZE, FOOT_H))
        foot.translate(-hd, sy * hw, FOOT_H / 2.0)
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


# ---- caster dimensions (meters) ---------------------------------------------
# Heavy-duty rigid caster at each front corner. The fork bracket extends
# outward from the body corner so the wheel rolls outside the body footprint.
CASTER_WHEEL_R = 0.050        # 100 mm dia wheel (compact industrial caster)
CASTER_WHEEL_W = 0.040        # 40 mm wide rubber wheel
CASTER_PLATE = 0.100          # 100 mm square mounting plate
CASTER_PLATE_T = 0.008        # plate thickness
CASTER_FORK_ARM_T = 0.010     # fork arm thickness (Y)
CASTER_FORK_ARM_DX = 0.040    # fork arm depth (X, front-to-back)
CASTER_FORK_GAP = 0.004       # gap between wheel side and fork arm
CASTER_STEM_R = 0.014         # kingpin stem radius
CASTER_STEM_H = 0.018         # stem height below plate
CASTER_ARM_W = 0.050          # horizontal bracket arm width
CASTER_ARM_T = 0.016          # bracket arm thickness (Z)

# front corner caster position: wheel center placed outside the body walls
# so the tire clears the body shell at the corner.
CASTER_X = BODY_D_BOT / 2.0 + 0.040   # 40 mm past the front wall
CASTER_Y = BODY_W / 2.0 + 0.060       # 60 mm past the side wall


def _caster_fork_mesh(sign_y: float) -> MeshGeometry:
    """Caster mounting bracket and fork arms at one front corner.

    Built in the body frame. ``sign_y`` is +1 or -1 for the ±Y corner.
    The bracket extends from the body underside outward to the wheel position
    so the wheel clears the body walls.  Adjacent components overlap slightly
    to guarantee mesh connectivity.
    """
    geo = MeshGeometry()
    # wheel center in the body frame
    wx = CASTER_X
    wy = sign_y * CASTER_Y

    # mounting plate at the body underside, near the corner but inside the body
    plate_cx = BODY_D_BOT / 2.0 - CASTER_PLATE / 2.0 - 0.01
    plate_cy = sign_y * (BODY_W / 2.0 - CASTER_PLATE / 2.0 - 0.01)
    plate_cz = FOOT_H - CASTER_PLATE_T / 2.0
    plate = BoxGeometry((CASTER_PLATE, CASTER_PLATE, CASTER_PLATE_T))
    plate.translate(plate_cx, plate_cy, plate_cz)
    geo.merge(plate)

    # horizontal bracket arm from the plate edge outward to above the wheel.
    # The arm Z is slightly below the plate bottom; overlap by 2 mm for
    # connectivity.
    arm_z_top = FOOT_H - CASTER_PLATE_T + 0.002     # embed 2 mm into the plate
    arm_z_bot = arm_z_top - CASTER_ARM_T
    arm_z = (arm_z_top + arm_z_bot) / 2.0
    # X-run: from plate center to wheel X
    xrun_len = abs(wx - plate_cx) + 0.01
    xrun = BoxGeometry((xrun_len, CASTER_ARM_W, CASTER_ARM_T))
    xrun.translate((plate_cx + wx) / 2.0, plate_cy, arm_z)
    geo.merge(xrun)
    # Y-run: from plate center to wheel Y
    yrun_len = abs(wy - plate_cy) + 0.01
    yrun = BoxGeometry((CASTER_ARM_W, yrun_len, CASTER_ARM_T))
    yrun.translate(plate_cx, (plate_cy + wy) / 2.0, arm_z)
    geo.merge(yrun)

    # gusset plate at the outer corner where the two arms meet (overlaps arms)
    gusset = BoxGeometry((CASTER_ARM_W + 0.01, CASTER_ARM_W + 0.01, CASTER_ARM_T + 0.004))
    gusset.translate(wx, wy, arm_z)
    geo.merge(gusset)

    # fork stem hanging from the gusset (overlaps gusset by 2 mm)
    stem_z_top = arm_z_bot + 0.002   # embed 2 mm into the gusset/arm
    stem_z_bot = stem_z_top - CASTER_STEM_H
    stem_cz = (stem_z_top + stem_z_bot) / 2.0
    stem = BoxGeometry((CASTER_STEM_R * 2, CASTER_STEM_R * 2, CASTER_STEM_H))
    stem.translate(wx, wy, stem_cz)
    geo.merge(stem)

    # fork yoke plate connecting stem to fork arms (overlaps stem by 2 mm)
    fork_half_spread = CASTER_WHEEL_W / 2.0 + CASTER_FORK_GAP + CASTER_FORK_ARM_T / 2.0
    yoke_z_top = stem_z_bot + 0.002
    yoke_z_bot = yoke_z_top - 0.008
    yoke_z = (yoke_z_top + yoke_z_bot) / 2.0
    yoke = BoxGeometry((CASTER_FORK_ARM_DX, 2.0 * fork_half_spread, 0.008))
    yoke.translate(wx, wy, yoke_z)
    geo.merge(yoke)

    # two fork arms extending down on either side of the wheel (overlaps yoke)
    arm_top_z = yoke_z_bot + 0.002   # embed 2 mm into the yoke
    arm_bot_z = CASTER_WHEEL_R - 0.008
    arm_h = max(arm_top_z - arm_bot_z, 0.012)
    arm_cz = (arm_top_z + arm_bot_z) / 2.0
    for sy in (1.0, -1.0):
        arm = BoxGeometry((CASTER_FORK_ARM_DX, CASTER_FORK_ARM_T, arm_h))
        arm.translate(wx, wy + sy * fork_half_spread, arm_cz)
        geo.merge(arm)

    # axle pin between the fork arms (overlaps fork arms by touching them)
    axle_len = 2.0 * fork_half_spread + CASTER_FORK_ARM_T
    axle = BoxGeometry((0.012, axle_len, 0.012))
    axle.translate(wx, wy, CASTER_WHEEL_R)
    geo.merge(axle)

    return geo


def _caster_wheel_mesh(prefix: str) -> tuple:
    """Build tire and hub meshes for one caster wheel.

    Returns (tire_managed_mesh, hub_managed_mesh).
    The wheel spins about local X (width axis); the part origin is at the axle
    center.
    """
    tire = TireGeometry(
        CASTER_WHEEL_R,
        CASTER_WHEEL_W,
        inner_radius=CASTER_WHEEL_R * 0.55,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
        tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.55),
        sidewall=TireSidewall(style="square", bulge=0.02),
    )
    hub = WheelGeometry(
        CASTER_WHEEL_R * 0.52,
        CASTER_WHEEL_W * 0.80,
        rim=WheelRim(
            inner_radius=CASTER_WHEEL_R * 0.28,
            flange_height=0.004,
            flange_thickness=0.003,
        ),
        hub=WheelHub(
            radius=CASTER_WHEEL_R * 0.18,
            width=CASTER_WHEEL_W * 0.90,
            cap_style="flat",
        ),
        bore=WheelBore(style="round", diameter=0.010),
    )
    return (
        mesh_from_geometry(tire, f"{prefix}_tire"),
        mesh_from_geometry(hub, f"{prefix}_hub"),
    )


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
    foot_dark = model.material("foot_steel", rgba=(0.14, 0.15, 0.14, 1.0))
    rubber = model.material("caster_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    caster_steel = model.material("caster_steel", rgba=(0.42, 0.44, 0.46, 1.0))

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

    # ---- front casters: fork + plate as body visuals, wheel as child parts --
    # Two front corners (+X side), mirrored in Y. Each gets a caster fork
    # (inline body visual) and a rolling wheel (separate part, CONTINUOUS joint).
    front_signs = [1.0, -1.0]
    for i in range(2):
        sy = front_signs[i]
        # fork + mounting plate as inline body visuals (non-moving decoration)
        fork_mesh = _caster_fork_mesh(sy)
        body.visual(
            mesh_from_geometry(fork_mesh, f"caster_fork_{i}"),
            material=caster_steel,
            name=f"caster_fork_{i}",
        )

        # wheel part: tire + hub visuals, origin at the axle center
        wheel_part = model.part(f"wheel_{i}")
        tire_mesh, hub_mesh = _caster_wheel_mesh(f"caster_{i}")
        # Wheel visuals are in the wheel part frame (origin at axle center).
        # TireGeometry and WheelGeometry spin about local X (width spans X).
        wheel_part.visual(tire_mesh, material=rubber, name=f"wheel_tire_{i}")
        wheel_part.visual(hub_mesh, material=caster_steel, name=f"wheel_hub_{i}")
        wheel_part.inertial = Inertial.from_geometry(
            Cylinder(radius=CASTER_WHEEL_R, length=CASTER_WHEEL_W),
            mass=2.5,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        # Continuous roll joint: wheel spins about its horizontal Y axle.
        # Joint origin at the axle center in the body frame (world coords
        # because body frame = world frame at rest).
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_part,
            origin=Origin(xyz=(CASTER_X, sy * CASTER_Y, CASTER_WHEEL_R)),
            # axis along Y: right-hand rule rolls the wheel forward/backward
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=12.0),
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

    # --- base feet sit at z ~ 0; overall dumpster size realistic -------------
    baabb = ctx.part_world_aabb(body)
    ctx.check(
        "base feet sit at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.005,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    bext = _ext(baabb)
    ctx.check(
        "dumpster is ~1.8 m wide (body walls; brackets extend further)",
        BODY_W - 0.02 < bext[1] < BODY_W + 0.30,
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

    # --- the slats read as one connected lid (no floating slats) -------------
    lid_slats = lid.get_visual("lid_slats")
    ctx.check("lid slats visual present", lid_slats is not None, "expected lid_slats")

    # --- front casters: two wheel parts with continuous roll joints ----------
    for i in range(2):
        wheel_part = object_model.get_part(f"wheel_{i}")
        wheel_joint = object_model.get_articulation(f"body_to_wheel_{i}")

        ctx.check(
            f"wheel_{i} part exists",
            wheel_part is not None,
            details=f"wheel_{i} not found",
        )
        ctx.check(
            f"wheel_{i} joint is continuous",
            str(wheel_joint.articulation_type).endswith("CONTINUOUS"),
            details=f"type={wheel_joint.articulation_type}",
        )
        ctx.check(
            f"wheel_{i} roll axis is horizontal Y (axle direction)",
            abs(wheel_joint.axis[1]) > 0.99
            and abs(wheel_joint.axis[0]) < 0.01
            and abs(wheel_joint.axis[2]) < 0.01,
            details=f"axis={wheel_joint.axis}",
        )
        # joint origin at the front (+X) corner, at the axle height
        ctx.check(
            f"wheel_{i} axle is at the front (+X) corner",
            wheel_joint.origin.xyz[0] > BODY_D_BOT / 2.0 - 0.15
            and wheel_joint.origin.xyz[2] < FOOT_H + 0.02,
            details=f"origin={wheel_joint.origin.xyz}",
        )

        # tire visual present on each wheel
        tire_vis = wheel_part.get_visual(f"wheel_tire_{i}")
        ctx.check(
            f"wheel_{i} has tire visual",
            tire_vis is not None,
            details=f"expected wheel_tire_{i}",
        )

        # caster fork visual present on the body
        fork_vis = body.get_visual(f"caster_fork_{i}")
        ctx.check(
            f"caster_fork_{i} visual present on body",
            fork_vis is not None,
            details=f"expected caster_fork_{i} on body",
        )

    # the wheel hubs nest within the fork arms (axle pin / hub overlap)
    for i in range(2):
        ctx.allow_overlap(
            body, f"wheel_{i}",
            elem_a=f"caster_fork_{i}",
            elem_b=f"wheel_hub_{i}",
            reason="The caster fork axle pin and hub are intentionally nested at the wheel axle bearing.",
        )
        ctx.allow_overlap(
            body, f"wheel_{i}",
            elem_a=f"caster_fork_{i}",
            elem_b=f"wheel_tire_{i}",
            reason="The caster fork axle pin passes through the wheel bore at the axle center.",
        )
        ctx.expect_contact(
            body, f"wheel_{i}",
            elem_a=f"caster_fork_{i}",
            elem_b=f"wheel_hub_{i}",
            name=f"wheel_{i} hub contacts the fork at the axle",
        )

    # wheel rolling: a pose change should rotate the wheel about Y
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_0 = object_model.get_part("wheel_0")
    rest_aabb_0 = ctx.part_world_aabb(wheel_0)
    with ctx.pose({wheel_joint_0: math.pi}):
        spun_aabb_0 = ctx.part_world_aabb(wheel_0)
    # AABB center should remain at the same position (wheel spins in place)
    if rest_aabb_0 and spun_aabb_0:
        rest_cz = (rest_aabb_0[0][2] + rest_aabb_0[1][2]) / 2.0
        spun_cz = (spun_aabb_0[0][2] + spun_aabb_0[1][2]) / 2.0
        ctx.check(
            "wheel_0 spins in place (center z stable under rotation)",
            abs(rest_cz - spun_cz) < 0.010,
            details=f"rest_cz={rest_cz}, spun_cz={spun_cz}",
        )

    return ctx.report()


object_model = build_object_model()
