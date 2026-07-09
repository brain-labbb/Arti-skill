from __future__ import annotations

# Small wooden keg / barrel with a side bunghole and removable bung plug.
#
# Real object (family fork of the parent slide-lid keg):
#   - A short, wide wooden keg: a slightly bulged barrel body built from
#     vertical staves, warm wood-grain coloring.
#   - Two dark metal hoop bands wrap the body (an upper band and a lower band).
#   - The top is a FIXED wooden head (permanent end head closing the mouth).
#   - A small round bunghole is bored through one stave near the mid-belly.
#   - A short tapered wooden bung plug seats into the bunghole.
#
# Coordinate convention:
#   - up is +Z. The barrel base footprint sits at z = 0.
#   - the body is a surface of revolution about the world Z axis.
#   - the bunghole is on the +X side at mid-belly height; the bung pulls
#     outward along +X (the local radial direction at the hole).
#
# Root structure: the staved barrel body (barrel_body) is the root. The two
# metal hoops and the fixed top head are attached to the body. The bung plug
# is the one moving part.
#
# PRIMARY ARTICULATION:
#   - body_to_bung : PRISMATIC, axis +X (radially outward at the bunghole).
#     At q=0 the bung is driven flush into the bunghole (seated); at the upper
#     limit (q=BUNG_TRAVEL) the bung has fully cleared the outer stave face.

import cadquery as cq
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
BODY_H = 0.360            # barrel body height (base at z=0, top at z=BODY_H)
R_END = 0.150             # outer radius at the top/bottom rims
R_MID = 0.186             # outer radius at the mid-bulge (slightly bulged)
WALL = 0.016              # stave wall thickness (body is hollow)
FLOOR_T = 0.020           # solid wooden base thickness (closes the bottom)

N_STAVES = 16             # number of suggested vertical staves
GROOVE_W = 0.0045         # vertical seam-groove width between staves
GROOVE_DEPTH = 0.006      # groove depth carved into the outer surface

UPPER_HOOP_Z = 0.262      # center height of the upper metal hoop
LOWER_HOOP_Z = 0.072      # center height of the lower metal hoop
HOOP_HALF_H = 0.022       # half-height of a hoop band
HOOP_PROUD = 0.007        # how far the hoop stands proud of the wood surface

HEAD_T = 0.018            # fixed top head (end head) thickness

BUNG_HOLE_R = 0.015       # bunghole bore radius
BUNG_Z = BODY_H / 2.0    # bunghole center height (mid-belly, between hoops)
BUNG_R_OUTER = 0.015      # bung outer face radius (matches hole for flush fit)
BUNG_R_INNER = 0.012      # bung inner tip radius (tapered for wedging)
BUNG_LENGTH = 0.030       # bung plug length (extends through wall + into cavity)
BUNG_TRAVEL = 0.065       # prismatic travel: 0 (closed/seated) -> fully pulled out

# Hoop configs for loop-based construction
HOOP_CONFIGS = [
    ("hoop_0", LOWER_HOOP_Z),
    ("hoop_1", UPPER_HOOP_Z),
]


def _outer_radius(z: float) -> float:
    """Outer barrel radius at height z: symmetric parabolic mid-bulge."""
    t = z / BODY_H
    return R_END + (R_MID - R_END) * (1.0 - (2.0 * t - 1.0) ** 2)


def _build_body_mesh():
    """Bulged barrel shell with a SOLID bottom, vertical stave grooves, and a
    bunghole bored through one stave at mid-belly along +X."""
    zs_outer = np.linspace(0.0, BODY_H, 13)
    outer = [(_outer_radius(z), z) for z in zs_outer]
    # Inner cavity wall: from the top down only to the floor level.
    zs_inner = np.linspace(BODY_H, FLOOR_T, 12)
    inner = [(_outer_radius(z) - WALL, z) for z in zs_inner]
    # Close in across the cavity floor to the axis, then down the centerline.
    pts = outer + inner + [(0.0, FLOOR_T), (0.0, 0.0)]

    wp = cq.Workplane("XZ").moveTo(*pts[0])
    for p in pts[1:]:
        wp = wp.lineTo(*p)
    # Revolve the (radius, height) loop 360 deg about the world Z axis.
    shell = wp.close().revolve(360, (0, 0, 0), (0, 1, 0))

    # Carve thin vertical seam grooves to suggest individual staves.
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

    # Bore the bunghole through the wall at mid-belly, along +X direction.
    # The cutter cylinder starts inside the cavity and extends past the outer face.
    hole_cutter = (
        cq.Workplane("YZ")
        .circle(BUNG_HOLE_R)
        .extrude(WALL + 0.02)
        .translate((_outer_radius(BUNG_Z) - WALL - 0.01, 0.0, BUNG_Z))
    )
    body = body.cut(hole_cutter)

    return mesh_from_cadquery(body, "barrel_body", tolerance=0.0015)


def _build_head_mesh():
    """Fixed wooden end head closing the top mouth of the barrel."""
    r = _outer_radius(BODY_H)
    head = (
        cq.Workplane("XY")
        .circle(r)
        .extrude(HEAD_T)
        .translate((0.0, 0.0, BODY_H))
    )
    # Slight chamfer on the top outer edge so it reads like a turned wood head.
    head = head.edges(">Z").fillet(0.004)
    return mesh_from_cadquery(head, "top_head", tolerance=0.0015)


def _build_hoop_mesh(name: str, z_center: float):
    """A dark metal hoop band wrapping the body, standing slightly proud."""
    r_body = _outer_radius(z_center)
    r_in = r_body - 0.001                                  # bite into the wood
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


def _build_bung_mesh():
    """Tapered wooden bung plug. Authored in the bung-local frame with the
    outer face center at local origin (x=0). The plug body extends in -X
    (into the bunghole when seated)."""
    # Build the tapered plug along +Z, then rotate to align with +X.
    # Inner tip at z=0, outer face at z=BUNG_LENGTH.
    bung = (
        cq.Workplane("XY")
        .circle(BUNG_R_INNER)
        .workplane(offset=BUNG_LENGTH)
        .circle(BUNG_R_OUTER)
        .loft()
    )
    # Rotate: +Z maps to +X via -90° around Y.
    bung = bung.rotate((0, 0, 0), (0, 1, 0), -90)
    # Shift so the outer face (was at z=BUNG_LENGTH -> x=BUNG_LENGTH) lands at x=0.
    bung = bung.translate((-BUNG_LENGTH, 0.0, 0.0))
    return mesh_from_cadquery(bung, "bung_plug", tolerance=0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_keg_barrel")

    model.material("oak_wood", rgba=(0.74, 0.52, 0.32, 1.0))
    model.material("head_wood", rgba=(0.78, 0.58, 0.36, 1.0))
    model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("bung_wood", rgba=(0.82, 0.64, 0.42, 1.0))

    # --- staved barrel body (root) -------------------------------------------
    body = model.part("barrel_body")
    body.visual(_build_body_mesh(), material="oak_wood", name="staved_shell")
    body.visual(_build_head_mesh(), material="head_wood", name="top_head")
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_MID, length=BODY_H + HEAD_T),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + HEAD_T) / 2.0)),
    )

    # --- two metal hoop bands (fixed to the body, loop-emitted) --------------
    hoop_parts = {}
    for i, (hoop_name, z_c) in enumerate(HOOP_CONFIGS):
        hoop = model.part(hoop_name)
        hoop.visual(_build_hoop_mesh(hoop_name, z_c), material="dark_metal", name=hoop_name)
        hoop.inertial = Inertial.from_geometry(
            Cylinder(radius=_outer_radius(z_c) + HOOP_PROUD, length=2 * HOOP_HALF_H),
            mass=0.4,
            origin=Origin(xyz=(0.0, 0.0, z_c)),
        )
        hoop_parts[hoop_name] = hoop

    # --- tapered wooden bung plug (the moving part) --------------------------
    # The bung mesh has its outer face center at local origin (x=0). The plug
    # extends in -X. The joint frame is placed on the outer stave face at the
    # bunghole so that at q=0 the bung outer face is flush with the stave.
    bung = model.part("bung_plug")
    bung.visual(_build_bung_mesh(), material="bung_wood", name="bung_plug")
    bung.inertial = Inertial.from_geometry(
        Cylinder(radius=BUNG_R_OUTER, length=BUNG_LENGTH),
        mass=0.05,
        origin=Origin(xyz=(-BUNG_LENGTH / 2.0, 0.0, 0.0)),
    )

    # --- articulations -------------------------------------------------------
    # Hoops: FIXED to body
    for i, (hoop_name, z_c) in enumerate(HOOP_CONFIGS):
        model.articulation(
            f"body_to_{hoop_name}",
            ArticulationType.FIXED,
            parent=body,
            child=hoop_parts[hoop_name],
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    # PRIMARY: bung slides radially outward from the bunghole.
    # Joint frame at the outer stave face at the hole; axis +X (radially out).
    # At q=0 the bung is seated flush; positive q pulls it outward.
    bung_r = _outer_radius(BUNG_Z)
    model.articulation(
        "body_to_bung",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bung,
        origin=Origin(xyz=(bung_r, 0.0, BUNG_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=BUNG_TRAVEL, effort=30.0, velocity=0.15
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    bung = object_model.get_part("bung_plug")
    bung_joint = object_model.get_articulation("body_to_bung")

    hoop_parts = [object_model.get_part(n) for n, _ in HOOP_CONFIGS]

    # --- PRIMARY articulation: prismatic, axis +X (horizontal radial) --------
    ctx.check(
        "bung joint is prismatic",
        bung_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={bung_joint.articulation_type}",
    )
    ax = tuple(bung_joint.axis)
    ctx.check(
        "bung joint axis is horizontal radial (+X)",
        ax[0] > 0.0 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = bung_joint.motion_limits
    ctx.check(
        "bung travel range is positive",
        lim is not None and lim.lower == 0.0 and lim.upper > 0.02,
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    # --- base sits at z = 0 --------------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "barrel base sits at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-3,
        details=f"body z-min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- body is hollow (thin shell) -----------------------------------------
    inner_r = _outer_radius(BODY_H) - WALL
    ctx.check(
        "body wall is a thin hollow shell",
        WALL < 0.5 * _outer_radius(BODY_H) and inner_r > 0.10,
        details=f"wall={WALL}, inner_r={inner_r:.3f}",
    )

    # --- bunghole pierces body wall (by construction) ------------------------
    bung_r_outer = _outer_radius(BUNG_Z)
    ctx.check(
        "bunghole pierces body wall",
        (BUNG_HOLE_R > 0
         and BUNG_HOLE_R < WALL * 3
         and BUNG_Z > FLOOR_T + 0.02
         and BUNG_Z < BODY_H - 0.02
         and bung_r_outer - WALL > 0),
        details=(
            f"hole_r={BUNG_HOLE_R}, wall={WALL}, bung_z={BUNG_Z}, "
            f"inner_r_at_hole={bung_r_outer - WALL:.4f}"
        ),
    )

    # --- materials present (wood variants + dark metal) ----------------------
    mat_names = {m.name for m in object_model.materials}
    ctx.check(
        "wood, head-wood, bung-wood, and dark-metal materials present",
        {"oak_wood", "head_wood", "dark_metal", "bung_wood"}.issubset(mat_names),
        details=f"materials={sorted(mat_names)}",
    )

    # --- both hoops present and wrapping the body ----------------------------
    for hoop, (hoop_name, z_c) in zip(hoop_parts, HOOP_CONFIGS):
        h_aabb = ctx.part_world_aabb(hoop)
        ctx.check(
            f"{hoop_name} present at expected height",
            h_aabb is not None and abs((h_aabb[0][2] + h_aabb[1][2]) / 2.0 - z_c) < 0.01,
            details=f"{hoop_name} aabb={h_aabb}",
        )
        ctx.expect_overlap(
            hoop, body, axes="z", min_overlap=0.01,
            name=f"{hoop_name} wraps the body (z overlap)",
        )

    # --- closed pose: bung seated flush in the bunghole ----------------------
    # At q=0 the bung outer face is at the stave outer surface (flush).
    # The bung body extends inward through the wall into the cavity.
    bung_pos = ctx.part_world_position(bung)
    ctx.check(
        "closed bung origin is at the outer stave face",
        bung_pos is not None
        and abs(bung_pos[0] - bung_r_outer) < 0.005
        and abs(bung_pos[1]) < 0.005
        and abs(bung_pos[2] - BUNG_Z) < 0.005,
        details=f"bung_pos={bung_pos}, expected~({bung_r_outer:.3f}, 0, {BUNG_Z:.3f})",
    )

    # The bung passes through the bunghole (which is a cut in the body wall),
    # so it reads as "floating" by mesh contact - but it's intentionally seated
    # in the hole. Allow it as isolated with proof checks.
    ctx.allow_isolated_part(
        bung,
        reason="The bung plug is intentionally seated inside the bunghole cut in the body wall; no mesh contact because the hole is a through-cut.",
    )
    # The bung is intentionally inside the body wall (seated in the bunghole).
    ctx.allow_overlap(
        bung, body,
        elem_a="bung_plug",
        elem_b="staved_shell",
        reason="The bung plug is intentionally seated inside the body wall bunghole at the closed pose.",
    )
    # Prove the bung is actually overlapping the body wall region (seated).
    ctx.expect_overlap(
        bung, body, axes="x", min_overlap=0.005,
        elem_a="bung_plug",
        elem_b="staved_shell",
        name="closed bung overlaps body wall along radial axis (seated in hole)",
    )

    # --- open pose: bung clears the outer stave face -------------------------
    rest_pos = ctx.part_world_position(bung)
    with ctx.pose({bung_joint: BUNG_TRAVEL}):
        open_pos = ctx.part_world_position(bung)
        open_aabb = ctx.part_world_aabb(bung)

    ctx.check(
        "bung moves radially outward to open (no lateral drift)",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] - rest_pos[0] > 0.02
        and abs(open_pos[1] - rest_pos[1]) < 1e-6
        and abs(open_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )
    ctx.check(
        "open bung inner tip clears outer stave face",
        open_aabb is not None and open_aabb[0][0] > bung_r_outer - 0.002,
        details=(
            f"open bung x-min={None if open_aabb is None else open_aabb[0][0]:.4f}, "
            f"stave outer x={bung_r_outer:.4f}"
        ),
    )

    # --- hoop overlap allowances (real coopered hoop fit) --------------------
    for hoop, (hoop_name, _zc) in zip(hoop_parts, HOOP_CONFIGS):
        ctx.allow_overlap(
            hoop, body,
            reason=f"The {hoop_name} metal hoop intentionally embeds slightly into the wood staves where it wraps the body.",
        )

    return ctx.report()


object_model = build_object_model()
