from __future__ import annotations

# Open hooded-top public trash can.
#
# Real object: a public / street trash can with a cylindrical black plastic
# body and a sheet-metal hood/canopy on top. The hood has walls on three sides
# plus a front aperture frame, with a peaked arch crown. The front aperture is
# a genuine open void into the hollow bin interior. A flat swing flap covers
# the aperture, hinged along its upper edge, swinging outward and up.
#
# Coordinate convention:
#   +Z is up; the body footprint sits at z = 0.
#   +X is the front (aperture side).
#   Hood aperture faces +X; flap hinge axis is horizontal (Y).
#
# Root: body. Hood is FIXED to body. Flap is REVOLUTE on hood.

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
R_BOTTOM = 0.150            # body radius at the floor
R_TOP = 0.170               # body radius at the mouth
BODY_H = 0.650              # body height
WALL_T = 0.005
FLOOR_T = 0.010

HOOD_W = 0.360              # hood width (Y)
HOOD_D = 0.360              # hood depth (X)
HOOD_H = 0.220              # hood wall height
SHEET_T = 0.003             # sheet metal thickness
ARCH_RISE = 0.040           # arch peak rise above wall tops

AP_W = 0.240                # aperture width
AP_H = 0.170                # aperture height
AP_SILL = 0.030             # sill height above hood base

FLAP_T = 0.004              # flap panel thickness
FLAP_CLEAR = 0.004          # clearance per edge

RIVET_R = 0.005             # rivet radius
RIVET_H = 0.004             # rivet protrusion
N_RIVETS = 6                # rivets along the lintel

# derived
AP_TOP_Z = BODY_H + AP_SILL + AP_H   # 0.850 — hinge / lintel bottom
HOOD_TOP_Z = BODY_H + HOOD_H         # 0.870 — wall tops
FLAP_W = AP_W - 2 * FLAP_CLEAR       # 0.232
FLAP_H = AP_H - 2 * FLAP_CLEAR       # 0.162


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
        outer, inner, segments=64, start_cap="flat", end_cap="flat",
    )
    floor = CylinderGeometry(R_BOTTOM - WALL_T, FLOOR_T, radial_segments=64)
    floor.translate(0.0, 0.0, FLOOR_T / 2.0)
    shell.merge(floor)
    return shell


def _rivet_geo() -> CylinderGeometry:
    """Shared rivet geometry helper: small cylindrical rivet (along Z)."""
    return CylinderGeometry(RIVET_R, RIVET_H, radial_segments=12)


def _hood_cq():
    """Sheet-metal hood: hollow box open at bottom with front aperture + arch."""
    T = SHEET_T

    # Outer solid box
    outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .box(HOOD_D, HOOD_W, HOOD_H, centered=(True, True, False))
    )
    # Interior cavity (leaves T-thick walls + T-thick top, open from bottom)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .box(HOOD_D - 2 * T, HOOD_W - 2 * T, HOOD_H - T,
             centered=(True, True, False))
    )
    hood = outer.cut(cavity)

    # Aperture cutter: box through the front wall
    ap_cutter = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H + AP_SILL)
        .center(HOOD_D / 2.0, 0.0)
        .box(T * 4, AP_W, AP_H, centered=(True, True, False))
    )
    hood = hood.cut(ap_cutter)

    # Peaked arch crown on top of the hood
    base_z = HOOD_TOP_Z
    hd = HOOD_D / 2.0
    hw = HOOD_W / 2.0
    arch = (
        cq.Workplane("XZ")
        .workplane(offset=-hw)
        .moveTo(-hd, base_z - T)          # overlap 1 layer into hood top
        .lineTo(0.0, base_z + ARCH_RISE)
        .lineTo(hd, base_z - T)
        .close()
        .extrude(HOOD_W)
    )
    hood = hood.union(arch)
    return hood


def _mounting_plate_mesh() -> MeshGeometry:
    """Flat annular mounting plate bridging body rim to hood walls.

    Sits just below the hood base (BODY_H - SHEET_T to BODY_H), inner edge
    overlaps the body wall by ~1 mm for a secure structural connection.
    """
    inner_r = R_TOP - 0.001          # overlaps body outer (R_TOP) by 1 mm
    outer_r = min(HOOD_D, HOOD_W) / 2.0   # reaches hood wall centre-line
    T = SHEET_T
    z_top = BODY_H
    z_bot = BODY_H - T
    seg = 48
    geo = MeshGeometry()

    ot, ob, it, ib = [], [], [], []
    for j in range(seg):
        th = 2.0 * math.pi * j / seg
        c, s = math.cos(th), math.sin(th)
        ot.append(geo.add_vertex(outer_r * c, outer_r * s, z_top))
        ob.append(geo.add_vertex(outer_r * c, outer_r * s, z_bot))
        it.append(geo.add_vertex(inner_r * c, inner_r * s, z_top))
        ib.append(geo.add_vertex(inner_r * c, inner_r * s, z_bot))

    for j in range(seg):
        jn = (j + 1) % seg
        # top annular face
        geo.add_face(ot[j], ot[jn], it[jn])
        geo.add_face(ot[j], it[jn], it[j])
        # bottom annular face (flipped)
        geo.add_face(ob[j], ib[jn], ob[jn])
        geo.add_face(ob[j], ib[j], ib[jn])
        # outer rim
        geo.add_face(ot[j], ob[j], ob[jn])
        geo.add_face(ot[j], ob[jn], ot[jn])
        # inner rim
        geo.add_face(it[j], it[jn], ib[jn])
        geo.add_face(it[j], ib[jn], ib[j])
    return geo


# ---- build -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hooded_trash_can")

    black = model.material("can_black", rgba=(0.10, 0.10, 0.11, 1.0))
    galv = model.material("hood_galvanized", rgba=(0.62, 0.64, 0.66, 1.0))
    alum = model.material("flap_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark = model.material("dark_steel", rgba=(0.35, 0.37, 0.39, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell(), "body_shell"),
        material=black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_TOP, length=BODY_H),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.45)),
    )

    # ---- hood (fixed to body) -----------------------------------------------
    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_hood_cq(), "hood_shell"),
        material=galv,
        name="hood_shell",
    )
    hood.visual(
        mesh_from_geometry(_mounting_plate_mesh(), "mounting_plate"),
        material=galv,
        name="mounting_plate",
    )
    hood.inertial = Inertial.from_geometry(
        Box((HOOD_D, HOOD_W, HOOD_H + ARCH_RISE)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_H + (HOOD_H + ARCH_RISE) * 0.5)),
    )

    # rivets along the aperture lintel (repeated sub-parts via loop)
    lintel_z = AP_TOP_Z + (HOOD_TOP_Z - AP_TOP_Z) / 2.0
    for i in range(N_RIVETS):
        y = -HOOD_W / 2.0 + HOOD_W * (i + 0.5) / N_RIVETS
        rivet = _rivet_geo()
        rivet.rotate_y(math.pi / 2.0)          # cylinder Z → X
        rivet.translate(HOOD_D / 2.0 + RIVET_H / 2.0, y, lintel_z)
        hood.visual(
            mesh_from_geometry(rivet, f"rivet_{i}"),
            material=dark,
            name=f"rivet_{i}",
        )

    model.articulation(
        "body_to_hood",
        ArticulationType.FIXED,
        parent=body,
        child=hood,
        origin=Origin(),
    )

    # ---- flap (revolute on hood) --------------------------------------------
    flap = model.part("flap")
    flap.visual(
        Box((FLAP_T, FLAP_W, FLAP_H)),
        origin=Origin(xyz=(-FLAP_T / 2.0, 0.0, -FLAP_H / 2.0)),
        material=alum,
        name="flap_panel",
    )
    # small push-handle on the outside, near the bottom
    flap.visual(
        Box((0.006, 0.060, 0.010)),
        origin=Origin(xyz=(0.003, 0.0, -FLAP_H * 0.40)),
        material=dark,
        name="flap_handle",
    )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_T, FLAP_W, FLAP_H)),
        mass=0.08,
        origin=Origin(xyz=(-FLAP_T / 2.0, 0.0, -FLAP_H / 2.0)),
    )

    # Hinge at the top of the aperture on the hood front face.
    # axis = (0, -1, 0): positive q swings the bottom of the flap outward (+X).
    model.articulation(
        "hood_to_flap",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=flap,
        origin=Origin(xyz=(HOOD_D / 2.0, 0.0, AP_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=1.3,
        ),
    )

    return model


# ---- helpers for tests -------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ---- tests -------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    flap = object_model.get_part("flap")
    flap_joint = object_model.get_articulation("hood_to_flap")

    # --- body scale & position -----------------------------------------------
    baabb = ctx.part_world_aabb(body)
    bext = _ext(baabb)
    ctx.check(
        "body base sits at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.004,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    ctx.check(
        "body height ~0.65 m",
        abs(bext[2] - BODY_H) < 0.02,
        details=f"body_ext={bext}",
    )
    ctx.check(
        "body is a ~0.34 m diameter cylinder",
        0.28 < bext[0] < 0.38 and 0.28 < bext[1] < 0.38,
        details=f"body_ext={bext}",
    )

    # --- hood sits on body, correct size -------------------------------------
    haabb = ctx.part_world_aabb(hood)
    hext = _ext(haabb)
    ctx.check(
        "hood crown rises above body rim",
        haabb[1][2] > BODY_H + HOOD_H + ARCH_RISE * 0.8,
        details=f"hood_top={haabb[1][2]}, expected>{BODY_H + HOOD_H + ARCH_RISE * 0.8}",
    )
    ctx.check(
        "hood footprint ~0.36 x 0.36 m",
        abs(hext[0] - HOOD_D) < 0.04 and abs(hext[1] - HOOD_W) < 0.04,
        details=f"hood_ext={hext}",
    )
    ctx.expect_overlap(
        hood, body, axes="xy", min_overlap=0.20,
        name="hood seats over the body mouth footprint",
    )
    # The mounting plate intentionally overlaps the body rim by ~1 mm for
    # a secure structural connection.
    ctx.allow_overlap(
        body, hood,
        elem_a="body_shell", elem_b="mounting_plate",
        reason="Mounting plate sleeves 1 mm over the body rim to anchor the hood to the body.",
    )
    ctx.expect_contact(
        body, hood,
        elem_a="body_shell", elem_b="mounting_plate",
        name="mounting plate contacts body rim",
    )

    # --- hood aperture exists: arch crown visible ----------------------------
    ctx.check(
        "arch crown extends above wall tops",
        haabb[1][2] > HOOD_TOP_Z + ARCH_RISE * 0.7,
        details=f"hood_top={haabb[1][2]}, wall_top={HOOD_TOP_Z}",
    )

    # --- rivets present on the lintel ----------------------------------------
    rivet_ok = True
    for i in range(N_RIVETS):
        vaabb = ctx.part_element_world_aabb(hood, elem=f"rivet_{i}")
        if vaabb is None:
            rivet_ok = False
            break
        # rivet should be on the +X face of the hood, near lintel height
        if vaabb[1][0] < HOOD_D / 2.0:
            rivet_ok = False
    ctx.check(
        f"all {N_RIVETS} lintel rivets present on hood front face",
        rivet_ok,
        details="one or more rivets missing or misplaced",
    )

    # --- flap covers the aperture at rest ------------------------------------
    faabb = ctx.part_world_aabb(flap)
    fext = _ext(faabb)
    ctx.check(
        "flap width matches aperture",
        abs(fext[1] - FLAP_W) < 0.01,
        details=f"flap_ext={fext}",
    )
    ctx.check(
        "flap height matches aperture",
        abs(fext[2] - FLAP_H) < 0.01,
        details=f"flap_ext={fext}",
    )
    ctx.check(
        "flap seated at hood front face",
        abs(faabb[1][0] - HOOD_D / 2.0) < 0.015,
        details=f"flap_max_x={faabb[1][0]}, hood_front={HOOD_D/2.0}",
    )
    # flap should overlap the aperture region in YZ to prove it covers the hole
    ctx.expect_overlap(
        flap, hood, axes="yz", min_overlap=0.05,
        name="flap covers the hood aperture at rest",
    )

    # --- primary articulation: REVOLUTE, axis lateral (Y) --------------------
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
        "flap outward-swing travel >= ~60 deg",
        lim is not None and lim.upper is not None and lim.upper >= 1.0,
        details=f"upper={lim.upper}",
    )

    # --- decisive pose: flap swings outward (+X) when opened -----------------
    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap swings outward: max-X increases when opened",
        open_aabb[1][0] > rest_aabb[1][0] + 0.02,
        details=f"rest_max_x={rest_aabb[1][0]}, open_max_x={open_aabb[1][0]}",
    )
    ctx.check(
        "flap footprint shrinks in Z when swung open",
        (open_aabb[1][2] - open_aabb[0][2]) < (rest_aabb[1][2] - rest_aabb[0][2]) - 0.02,
        details=(
            f"rest_z_ext={rest_aabb[1][2]-rest_aabb[0][2]}, "
            f"open_z_ext={open_aabb[1][2]-open_aabb[0][2]}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
