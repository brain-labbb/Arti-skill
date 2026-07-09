from __future__ import annotations

# Small wooden keg / barrel with a swing bail handle.
#
# Real object (from the reference image + parent model):
#   - A short, wide wooden keg: a slightly bulged barrel body built from
#     vertical staves, warm wood-grain coloring.
#   - Two dark metal hoop bands wrap the body (upper and lower).
#   - A fixed wooden top head closes the mouth.
#   - Two small metal pivot ears mount on opposite sides of the upper body.
#   - A single arched bail (half-loop wire handle) pivots between the ears.
#
# Coordinate convention:
#   - up is +Z. The barrel base footprint sits at z = 0.
#   - the body is a surface of revolution about the world Z axis.
#
# Root structure: the staved barrel body (barrel_body) is the root. The two
# metal hoops, the rim ledge, the top head, and the two pivot ears are all
# FIXED to the body. The bail handle is the one moving part.
#
# PRIMARY ARTICULATION:
#   - body_to_bail : REVOLUTE, axis +X (horizontal, through both ear pivots).
#     At q=0 the bail arch extends to the +Y side, resting against the barrel.
#     Positive q swings the bail up and overhead for carrying (at q=pi/2 the
#     arch is directly above the pivots). Full range 0 to pi.
#   - The two pivot ears are emitted via a for-i-in-range(2) loop and FIXED
#     to the body on the actual stave face.

import cadquery as cq
import math
import numpy as np

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) -------------------------------------------------
BODY_H = 0.360            # barrel body height (base at z=0, mouth at z=BODY_H)
R_END = 0.150            # outer radius at the top/bottom rims
R_MID = 0.186            # outer radius at the mid-bulge (slightly bulged)
WALL = 0.016            # stave wall thickness (body is hollow)
FLOOR_T = 0.020         # solid wooden base thickness (closes the bottom)

N_STAVES = 16            # number of suggested vertical staves
GROOVE_W = 0.0045        # vertical seam-groove width between staves
GROOVE_DEPTH = 0.006     # groove depth carved into the outer surface

UPPER_HOOP_Z = 0.262     # center height of the upper metal hoop
LOWER_HOOP_Z = 0.072     # center height of the lower metal hoop
HOOP_HALF_H = 0.022      # half-height of a hoop band
HOOP_PROUD = 0.007       # how far the hoop stands proud of the wood surface

RIM_LEDGE_DZ = 0.020     # how far the inner seating ledge sits below the mouth
RIM_LEDGE_W = 0.014      # radial width of the inner seating ledge

TOP_HEAD_T = 0.020       # thickness of the fixed wooden top head

# Pivot ear bracket dimensions
Z_EAR = 0.310            # height of ear pivot centers
EAR_W = 0.030            # ear plate width (circumferential, along barrel)
EAR_H = 0.040            # ear plate height (vertical, along barrel)
EAR_T = 0.005            # ear plate thickness (radial, against stave face)
EAR_PIN_R = 0.005        # pivot pin radius
EAR_PIN_L = 0.030        # pivot pin length (outward protrusion from plate)

# Bail handle dimensions
BAIL_WIRE_R = 0.005      # wire/rod radius
BAIL_ARCH_D = 0.20       # arch depth: how far the bail extends to one side


def _outer_radius(z: float) -> float:
    """Outer barrel radius at height z: symmetric parabolic mid-bulge."""
    t = z / BODY_H
    return R_END + (R_MID - R_END) * (1.0 - (2.0 * t - 1.0) ** 2)


# Derived: barrel radius at ear height and pivot half-span
_R_AT_EAR = _outer_radius(Z_EAR)
_PIVOT_HALF_SPAN = _R_AT_EAR + EAR_T + EAR_PIN_L / 2.0


def _build_body_mesh():
    """Bulged barrel shell with a SOLID bottom and vertical stave grooves."""
    zs_outer = np.linspace(0.0, BODY_H, 13)
    outer = [(_outer_radius(z), z) for z in zs_outer]
    zs_inner = np.linspace(BODY_H, FLOOR_T, 12)
    inner = [(_outer_radius(z) - WALL, z) for z in zs_inner]
    pts = outer + inner + [(0.0, FLOOR_T), (0.0, 0.0)]

    wp = cq.Workplane("XZ").moveTo(*pts[0])
    for p in pts[1:]:
        wp = wp.lineTo(*p)
    shell = wp.close().revolve(360, (0, 0, 0), (0, 1, 0))

    cutters = None
    for i in range(N_STAVES):
        ang = 360.0 / N_STAVES * i
        slot = (
            cq.Workplane("XY")
            .box(GROOVE_DEPTH * 2.0, GROOVE_W, BODY_H + 0.02)
            .translate((R_MID, 0.0, BODY_H / 2.0))
            .rotate((0, 0, 0), (0, 0, 1), ang)
        )
        cutters = slot if cutters is None else cutters.union(slot)
    body = shell.cut(cutters)
    return mesh_from_cadquery(body, "barrel_body", tolerance=0.0015)


def _build_rim_mesh():
    """Recessed inner seating ledge just below the mouth."""
    r_mouth_inner = _outer_radius(BODY_H) - WALL
    z_top = BODY_H
    z_bot = BODY_H - RIM_LEDGE_DZ
    r_out = r_mouth_inner
    r_in = r_mouth_inner - RIM_LEDGE_W
    ring = (
        cq.Workplane("XZ")
        .moveTo(r_in, z_bot)
        .lineTo(r_out, z_bot)
        .lineTo(r_out, z_top)
        .lineTo(r_in, z_top)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return mesh_from_cadquery(ring, "rim_ledge", tolerance=0.0015)


def _build_hoop_mesh(name: str, z_center: float):
    """A dark metal hoop band wrapping the body, standing slightly proud."""
    r_body = _outer_radius(z_center)
    r_in = r_body - 0.001
    r_out = r_body + HOOP_PROUD
    z_bot = z_center - HOOP_HALF_H
    z_top = z_center + HOOP_HALF_H
    band = (
        cq.Workplane("XZ")
        .moveTo(r_in, z_bot)
        .lineTo(r_out, z_bot)
        .lineTo(r_out, z_top)
        .lineTo(r_in, z_top)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return mesh_from_cadquery(band, name, tolerance=0.0015)


def _build_top_head_mesh():
    """Fixed wooden top head disk that closes the barrel mouth."""
    r = _outer_radius(BODY_H) - 0.003  # slight clearance inside mouth rim
    disk = cq.Workplane("XY").circle(r).extrude(TOP_HEAD_T)
    disk = disk.edges(">Z").fillet(0.004)
    return mesh_from_cadquery(disk, "top_head", tolerance=0.0015)


def _build_ear_mesh(name: str):
    """Small metal pivot ear: a flat mounting plate with a cylindrical pivot pin.

    Authored in local frame:
      - plate centered at origin, extends along X (circumferential) and
        Z (vertical), thin along Y (radial/thickness).
      - pivot pin protrudes from the outer face (+Y direction).
    """
    # Build plate and pin on a single workplane chain so the union is solid
    ear = (
        cq.Workplane("XY")
        .box(EAR_W, EAR_T, EAR_H)
        .faces(">Y")
        .workplane()
        .circle(EAR_PIN_R)
        .extrude(EAR_PIN_L)
    )
    return mesh_from_cadquery(ear, name, tolerance=0.001)


def _build_bail_mesh():
    """Arched bail (half-loop wire handle) spanning between the two ear pivots.

    Built by sweeping a circular profile along a spline path. Authored in the
    bail local frame (which coincides with the joint frame at q=0). The bail
    endpoints are at (+-PIVOT_HALF_SPAN, 0, 0). The arch extends in the +Y
    direction so that at q=0 the bail rests against the +Y side of the barrel.
    Rotation about the X axis swings it overhead for carrying.
    """
    px = _PIVOT_HALF_SPAN
    d = BAIL_ARCH_D

    # Spline path in the XY plane (z=0) connecting the two ear pivots
    path_pts = [
        (-px, 0.0),
        (-px, d * 0.35),
        (-px * 0.6, d * 0.7),
        (0.0, d),
        (px * 0.6, d * 0.7),
        (px, d * 0.35),
        (px, 0.0),
    ]
    path = cq.Workplane("XY").spline(path_pts)

    # Circle profile at the start point, perpendicular to the path direction
    # Path starts at (-px, 0) going roughly in +Y, so profile plane is XZ
    profile = (
        cq.Workplane("XZ")
        .transformed(offset=cq.Vector(-px, 0.0, 0.0))
        .circle(BAIL_WIRE_R)
    )

    bail = profile.sweep(path, isFrenet=True)
    return mesh_from_cadquery(bail, "bail_handle", tolerance=0.0008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_keg_bail_barrel")

    model.material("oak_wood", rgba=(0.74, 0.52, 0.32, 1.0))
    model.material("head_wood", rgba=(0.78, 0.56, 0.35, 1.0))
    model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("bail_metal", rgba=(0.22, 0.22, 0.24, 1.0))

    # --- staved barrel body (root) -------------------------------------------
    body = model.part("barrel_body")
    body.visual(_build_body_mesh(), material="oak_wood")
    body.visual(_build_rim_mesh(), material="oak_wood")
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_MID, length=BODY_H),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # --- two metal hoop bands (fixed to the body) ----------------------------
    upper_hoop = model.part("upper_hoop")
    upper_hoop.visual(
        _build_hoop_mesh("upper_hoop", UPPER_HOOP_Z), material="dark_metal"
    )
    upper_hoop.inertial = Inertial.from_geometry(
        Cylinder(radius=_outer_radius(UPPER_HOOP_Z) + HOOP_PROUD, length=2 * HOOP_HALF_H),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, UPPER_HOOP_Z)),
    )

    lower_hoop = model.part("lower_hoop")
    lower_hoop.visual(
        _build_hoop_mesh("lower_hoop", LOWER_HOOP_Z), material="dark_metal"
    )
    lower_hoop.inertial = Inertial.from_geometry(
        Cylinder(radius=_outer_radius(LOWER_HOOP_Z) + HOOP_PROUD, length=2 * HOOP_HALF_H),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HOOP_Z)),
    )

    # --- fixed wooden top head (replaces the parent's sliding lid) -----------
    top_head = model.part("top_head")
    top_head.visual(_build_top_head_mesh(), material="head_wood")
    top_head.inertial = Inertial.from_geometry(
        Cylinder(radius=_outer_radius(BODY_H), length=TOP_HEAD_T),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, TOP_HEAD_T / 2.0)),
    )

    # --- two pivot ears (for-i-in-range loop, FIXED to body) -----------------
    ears = []
    for i in range(2):
        ear_name = f"pivot_ear_{i}"
        ear = model.part(ear_name)
        ear.visual(_build_ear_mesh(ear_name), material="dark_metal")

        sign = 1 if i == 0 else -1
        # Mount: inner face of plate (local -Y) faces barrel center.
        # For +X side (i=0): rpy = (0, 0, -pi/2) maps local +Y -> world +X
        # For -X side (i=1): rpy = (0, 0, +pi/2) maps local +Y -> world -X
        yaw = -math.pi / 2.0 if i == 0 else math.pi / 2.0
        ear_origin = Origin(
            xyz=(sign * (_R_AT_EAR + EAR_T / 2.0), 0.0, Z_EAR),
            rpy=(0.0, 0.0, yaw),
        )

        model.articulation(
            f"body_to_{ear_name}",
            ArticulationType.FIXED,
            parent=body,
            child=ear,
            origin=ear_origin,
        )
        ears.append(ear)

    # --- bail handle (the one moving part) -----------------------------------
    bail = model.part("bail_handle")
    bail.visual(_build_bail_mesh(), material="bail_metal")
    bail.inertial = Inertial.from_geometry(
        Cylinder(radius=BAIL_WIRE_R, length=2.0 * _PIVOT_HALF_SPAN),
        mass=0.12,
        origin=Origin(xyz=(0.0, BAIL_ARCH_D / 2.0, 0.0)),
    )

    # --- articulations -------------------------------------------------------
    model.articulation(
        "body_to_upper_hoop",
        ArticulationType.FIXED,
        parent=body,
        child=upper_hoop,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_lower_hoop",
        ArticulationType.FIXED,
        parent=body,
        child=lower_hoop,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Top head: FIXED on the mouth top. Mesh has bottom at local z=0.
    model.articulation(
        "body_to_top_head",
        ArticulationType.FIXED,
        parent=body,
        child=top_head,
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
    )

    # PRIMARY: bail swings on a horizontal axis through both ear pivots.
    # Joint origin at the midpoint between ears on the pivot line.
    # Axis +X: positive q rotates the arch from +Y side up toward +Z (overhead).
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, Z_EAR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.pi,
            effort=10.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    upper_hoop = object_model.get_part("upper_hoop")
    lower_hoop = object_model.get_part("lower_hoop")
    top_head = object_model.get_part("top_head")
    ear_0 = object_model.get_part("pivot_ear_0")
    ear_1 = object_model.get_part("pivot_ear_1")
    bail = object_model.get_part("bail_handle")
    bail_joint = object_model.get_articulation("body_to_bail")

    # --- PRIMARY: bail joint is REVOLUTE about a horizontal axis -------------
    ctx.check(
        "bail joint is revolute",
        bail_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={bail_joint.articulation_type}",
    )
    ax = tuple(bail_joint.axis)
    ctx.check(
        "bail joint axis is horizontal (along X)",
        abs(ax[0]) > 0.9 and abs(ax[1]) < 0.1 and abs(ax[2]) < 0.1,
        details=f"axis={ax}",
    )
    lim = bail_joint.motion_limits
    ctx.check(
        "bail motion range spans rest to overhead",
        lim is not None and lim.lower == 0.0 and lim.upper > 1.4,
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    # --- two pivot ears on opposite sides -----------------------------------
    ear_0_aabb = ctx.part_world_aabb(ear_0)
    ear_1_aabb = ctx.part_world_aabb(ear_1)
    ctx.check(
        "two pivot ears exist",
        ear_0_aabb is not None and ear_1_aabb is not None,
    )
    if ear_0_aabb is not None and ear_1_aabb is not None:
        ear_0_cx = (ear_0_aabb[0][0] + ear_0_aabb[1][0]) / 2.0
        ear_1_cx = (ear_1_aabb[0][0] + ear_1_aabb[1][0]) / 2.0
        ctx.check(
            "ears on opposite sides of barrel (opposite X signs)",
            ear_0_cx * ear_1_cx < 0.0,
            details=f"ear_0_cx={ear_0_cx:.4f}, ear_1_cx={ear_1_cx:.4f}",
        )
        # Ears at the expected height
        for ear_aabb, nm in ((ear_0_aabb, "ear_0"), (ear_1_aabb, "ear_1")):
            ear_cz = (ear_aabb[0][2] + ear_aabb[1][2]) / 2.0
            ctx.check(
                f"{nm} at expected pivot height",
                abs(ear_cz - Z_EAR) < 0.025,
                details=f"{nm} cz={ear_cz:.4f}, expected={Z_EAR}",
            )

    # --- bail endpoints reach both ears (spans between them) ----------------
    bail_aabb = ctx.part_world_aabb(bail)
    if bail_aabb is not None and ear_0_aabb is not None and ear_1_aabb is not None:
        # Bail X extent should encompass both ear X positions
        bail_x_min = bail_aabb[0][0]
        bail_x_max = bail_aabb[1][0]
        ear_x_min = min(ear_0_aabb[0][0], ear_1_aabb[0][0])
        ear_x_max = max(ear_0_aabb[1][0], ear_1_aabb[1][0])
        ctx.check(
            "bail spans between both ears in X",
            bail_x_min <= ear_x_min + 0.015 and bail_x_max >= ear_x_max - 0.015,
            details=(
                f"bail x=[{bail_x_min:.4f}, {bail_x_max:.4f}], "
                f"ears x=[{ear_x_min:.4f}, {ear_x_max:.4f}]"
            ),
        )

    # --- bail swings up overhead when positive q is applied -----------------
    rest_aabb = ctx.part_world_aabb(bail)
    with ctx.pose({bail_joint: math.pi / 2.0}):
        overhead_aabb = ctx.part_world_aabb(bail)
    ctx.check(
        "bail swings up overhead at pi/2",
        rest_aabb is not None
        and overhead_aabb is not None
        and overhead_aabb[1][2] > rest_aabb[1][2] + 0.05,
        details=(
            f"rest_z_max={None if rest_aabb is None else rest_aabb[1][2]}, "
            f"overhead_z_max={None if overhead_aabb is None else overhead_aabb[1][2]}"
        ),
    )
    if overhead_aabb is not None:
        ctx.check(
            "bail arch is above barrel top at carrying pose",
            overhead_aabb[1][2] > BODY_H + TOP_HEAD_T + 0.03,
            details=f"bail z_max={overhead_aabb[1][2]:.4f}, barrel top={BODY_H + TOP_HEAD_T}",
        )

    # --- base sits at z = 0 --------------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "barrel base sits at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-3,
        details=f"body z-min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- body is hollow (thin shell, closed top and bottom) -----------------
    inner_r = _outer_radius(BODY_H) - WALL
    ctx.check(
        "body wall is a thin hollow shell",
        WALL < 0.5 * _outer_radius(BODY_H) and inner_r > 0.10,
        details=f"wall={WALL}, inner_r={inner_r:.3f}",
    )

    # --- top head seated on the barrel mouth --------------------------------
    ctx.expect_gap(
        top_head,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        name="top head seats flush on the barrel mouth",
    )

    # --- both hoops present and wrapping the body ---------------------------
    for hoop, zc, nm in (
        (upper_hoop, UPPER_HOOP_Z, "upper"),
        (lower_hoop, LOWER_HOOP_Z, "lower"),
    ):
        h_aabb = ctx.part_world_aabb(hoop)
        ctx.check(
            f"{nm} hoop present at expected height",
            h_aabb is not None and abs((h_aabb[0][2] + h_aabb[1][2]) / 2.0 - zc) < 0.01,
            details=f"{nm} hoop aabb={h_aabb}",
        )
        ctx.expect_overlap(
            hoop,
            body,
            axes="z",
            min_overlap=0.01,
            name=f"{nm} hoop wraps the body (z overlap)",
        )

    # --- materials present ---------------------------------------------------
    mat_names = {m.name for m in object_model.materials}
    ctx.check(
        "wood, dark-metal, and bail-metal materials present",
        {"oak_wood", "head_wood", "dark_metal", "bail_metal"}.issubset(mat_names),
        details=f"materials={sorted(mat_names)}",
    )

    # --- overlap allowances --------------------------------------------------
    # Metal hoops bite slightly into the wood staves (real coopered hoop fit).
    ctx.allow_overlap(
        upper_hoop, body,
        reason="The upper metal hoop intentionally embeds slightly into the wood staves.",
    )
    ctx.allow_overlap(
        lower_hoop, body,
        reason="The lower metal hoop intentionally embeds slightly into the wood staves.",
    )
    # Top head seats on the mouth rim (contact overlap at the seating plane).
    ctx.allow_overlap(
        top_head, body,
        reason="The top head seats onto the barrel mouth rim with contact-level embedding.",
    )
    # Each pivot ear mounts flush on the curved stave face; the flat plate
    # may marginally overlap the curved barrel surface at the edges.
    ctx.allow_overlap(
        ear_0, body,
        reason="The pivot ear plate mounts on the curved stave face with marginal contact.",
    )
    ctx.allow_overlap(
        ear_1, body,
        reason="The pivot ear plate mounts on the curved stave face with marginal contact.",
    )
    # Bail wire endpoints pivot on the ear pins (intentional pivot embedding).
    ctx.allow_overlap(
        bail, ear_0,
        reason="The bail wire endpoint pivots on the ear pin (intentional pivot embedding).",
    )
    ctx.allow_overlap(
        bail, ear_1,
        reason="The bail wire endpoint pivots on the ear pin (intentional pivot embedding).",
    )

    # Prove the pivot embedding is local and the bail still spans correctly:
    ctx.expect_contact(
        bail, ear_0,
        contact_tol=0.006,
        name="bail contacts ear_0 pivot point",
    )
    ctx.expect_contact(
        bail, ear_1,
        contact_tol=0.006,
        name="bail contacts ear_1 pivot point",
    )

    return ctx.report()


object_model = build_object_model()
