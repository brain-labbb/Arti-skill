from __future__ import annotations

# Small wooden keg barrel with staved bulged body, two dark metal hoop bands,
# a closed fixed top head, and two rotating metal side-ear carry rings.
#
# Real object:
#   - A short, wide wooden keg: a slightly bulged barrel body built from
#     vertical staves, warm wood-grain coloring.
#   - Two dark metal hoop bands wrap the body (upper and lower).
#   - A fixed wooden top head closes the mouth permanently.
#   - Two metal ear brackets are mounted on opposite sides of the body.
#   - Each ear carries a metal ring that pivots on a REVOLUTE joint:
#     the ring folds flat against the barrel at q=0 or swings outward
#     to be grabbed at positive q.
#
# Coordinate convention:
#   - up is +Z. The barrel base footprint sits at z = 0.
#   - the body is a surface of revolution about the world Z axis.
#
# Root structure: barrel_body is the root. Hoops, rim ledge, and top head
# are visuals on the body. Ear brackets are FIXED to the body. Each ring
# is REVOLUTE on its ear.
#
# ARTICULATIONS:
#   body_to_upper_hoop : FIXED
#   body_to_lower_hoop : FIXED
#   body_to_ear_0      : FIXED (ear bracket on +X barrel face)
#   body_to_ear_1      : FIXED (ear bracket on -X barrel face)
#   ear_0_to_ring_0    : REVOLUTE, axis (1,0,0) in ear frame, limits [0, 1.8]
#   ear_1_to_ring_1    : REVOLUTE, axis (1,0,0) in ear frame, limits [0, 1.8]
#
# Both ring joints share the same axis in the parent ear frame and the same
# motion limits. Positive q swings the ring outward from the barrel on both
# sides (because the ear frames face opposite directions in world).

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
BODY_H = 0.360            # barrel body height (base at z=0, mouth at z=BODY_H)
R_END = 0.150             # outer radius at the top/bottom rims
R_MID = 0.186             # outer radius at the mid-bulge
WALL = 0.016              # stave wall thickness
FLOOR_T = 0.020           # solid wooden base thickness

N_STAVES = 16
GROOVE_W = 0.0045
GROOVE_DEPTH = 0.006

UPPER_HOOP_Z = 0.262
LOWER_HOOP_Z = 0.072
HOOP_HALF_H = 0.022
HOOP_PROUD = 0.007

RIM_LEDGE_DZ = 0.020
RIM_LEDGE_W = 0.014

# --- ear bracket dimensions ---
EAR_PLATE_W = 0.025       # plate width (along barrel circumference)
EAR_PLATE_T = 0.020       # plate thickness (radial standoff from barrel)
EAR_PLATE_H = 0.030       # plate height (along barrel axis)
EAR_FORK_W = 0.005        # fork prong width
EAR_FORK_H = 0.014        # fork prong height (above plate top)
EAR_FORK_DEPTH = 0.010    # fork prong depth (outward from plate face)
EAR_FORK_GAP = 0.011      # gap between fork prongs
EAR_MOUNT_Z = 0.150       # bottom of ear plate on barrel

# --- ring dimensions ---
RING_MAJOR_R = 0.020      # ring major radius (center-to-tube-center)
RING_MINOR_R = 0.003      # ring tube radius

RING_UPPER = 1.80         # max ring swing angle (radians, ~103 degrees)


def _outer_radius(z: float) -> float:
    """Outer barrel radius at height z: symmetric parabolic mid-bulge."""
    t = z / BODY_H
    return R_END + (R_MID - R_END) * (1.0 - (2.0 * t - 1.0) ** 2)


def _build_body_mesh():
    """Bulged barrel shell with solid bottom and vertical stave grooves."""
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
    """Fixed wooden disk closing the barrel mouth permanently."""
    r_head = _outer_radius(BODY_H) - WALL - 0.002
    head = cq.Workplane("XY").circle(r_head).extrude(0.012)
    head = head.translate((0, 0, BODY_H - 0.012))
    return mesh_from_cadquery(head, "top_head", tolerance=0.0015)


def _build_ear_mesh(name: str):
    """Metal ear bracket: flat plate + two fork prongs at top.

    Local frame: X across plate, Y outward from barrel (plate normal), Z up.
    Plate back face at y=0. Fork prongs extend further in +Y from the plate
    top edge, with a gap between them for the carry ring pivot."""
    # Main plate
    plate = (
        cq.Workplane("XY")
        .box(EAR_PLATE_W, EAR_PLATE_T, EAR_PLATE_H)
        .translate((0.0, EAR_PLATE_T / 2.0, EAR_PLATE_H / 2.0))
    )
    # Fork prongs extending outward from plate top
    prong_y = EAR_PLATE_T + EAR_FORK_DEPTH / 2.0
    prong_z = EAR_PLATE_H + EAR_FORK_H / 2.0
    prong_x = (EAR_FORK_GAP + EAR_FORK_W) / 2.0

    left = (
        cq.Workplane("XY")
        .box(EAR_FORK_W, EAR_FORK_DEPTH, EAR_FORK_H)
        .translate((-prong_x, prong_y, prong_z))
    )
    right = (
        cq.Workplane("XY")
        .box(EAR_FORK_W, EAR_FORK_DEPTH, EAR_FORK_H)
        .translate((prong_x, prong_y, prong_z))
    )
    ear = plate.union(left).union(right)
    return mesh_from_cadquery(ear, name, tolerance=0.001)


def _build_ring_mesh(name: str):
    """Metal carry ring: torus in the XZ plane, hanging below the pivot.

    Local frame: pivot at origin, ring hangs in -Z. The torus lies in the
    XZ plane (symmetric around Y). The pivot axis is along X (through the
    fork prongs of the parent ear bracket).

    At q=0 the ring hangs flat against the barrel. Positive q around X
    swings the ring outward from the barrel surface."""
    R = RING_MAJOR_R
    r = RING_MINOR_R
    # Torus in XZ plane by revolving a circle around Y axis.
    # Circle center at (R, 0) in XZ workplane = world (X=R, Z=0).
    torus = (
        cq.Workplane("XZ")
        .moveTo(R, 0)
        .circle(r)
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    # Shift down so the top of the ring (pivot point) is at origin.
    ring = torus.translate((0, 0, -R))
    return mesh_from_cadquery(ring, name, tolerance=0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_keg_barrel")

    model.material("oak_wood", rgba=(0.74, 0.52, 0.32, 1.0))
    model.material("head_wood", rgba=(0.80, 0.60, 0.39, 1.0))
    model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))

    # --- staved barrel body (root) -------------------------------------------
    body = model.part("barrel_body")
    body.visual(_build_body_mesh(), material="oak_wood")
    body.visual(_build_rim_mesh(), material="oak_wood")
    body.visual(_build_top_head_mesh(), material="head_wood")
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

    lower_hoop = model.part("lower_hoop")
    lower_hoop.visual(
        _build_hoop_mesh("lower_hoop", LOWER_HOOP_Z), material="dark_metal"
    )

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

    # --- ear brackets and carry rings (for-i-in-range(2) loop) ---------------
    r_barrel_at_ear = _outer_radius(EAR_MOUNT_Z + EAR_PLATE_H / 2.0)
    pivot_y = EAR_PLATE_T + EAR_FORK_DEPTH / 2.0
    pivot_z = EAR_PLATE_H + EAR_FORK_H / 2.0

    ears = []
    rings = []

    for i in range(2):
        sign = 1 if i == 0 else -1
        yaw = -np.pi / 2.0 if i == 0 else np.pi / 2.0

        # Ear bracket (FIXED to body on the stave face)
        ear = model.part(f"ear_{i}")
        ear.visual(_build_ear_mesh(f"ear_{i}"), material="dark_metal")
        ears.append(ear)

        model.articulation(
            f"body_to_ear_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=ear,
            origin=Origin(
                xyz=(sign * r_barrel_at_ear, 0.0, EAR_MOUNT_Z),
                rpy=(0.0, 0.0, yaw),
            ),
        )

        # Carry ring (REVOLUTE on ear)
        ring = model.part(f"ring_{i}")
        ring.visual(_build_ring_mesh(f"ring_{i}"), material="dark_metal")
        rings.append(ring)

        model.articulation(
            f"ear_{i}_to_ring_{i}",
            ArticulationType.REVOLUTE,
            parent=ear,
            child=ring,
            origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0, upper=RING_UPPER, effort=5.0, velocity=2.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")

    # --- exactly two ring parts, each on its own REVOLUTE joint --------------
    ring_parts = [p for p in object_model.parts if p.name.startswith("ring_")]
    ctx.check(
        "exactly two ring parts",
        len(ring_parts) == 2,
        details=f"found {len(ring_parts)} ring parts: {[p.name for p in ring_parts]}",
    )

    for rp in ring_parts:
        ring_joint = None
        for art in object_model.articulations:
            child_name = art.child if isinstance(art.child, str) else art.child.name
            if child_name == rp.name:
                ring_joint = art
                break
        ctx.check(
            f"{rp.name} has a parent articulation",
            ring_joint is not None,
            details=f"no articulation found with child={rp.name}",
        )
        if ring_joint is not None:
            ctx.check(
                f"{rp.name} joint is REVOLUTE",
                ring_joint.articulation_type == ArticulationType.REVOLUTE,
                details=f"type={ring_joint.articulation_type}",
            )

    # --- two ears on opposite sides of the body ------------------------------
    ear_0 = object_model.get_part("ear_0")
    ear_1 = object_model.get_part("ear_1")
    pos_0 = ctx.part_world_position(ear_0)
    pos_1 = ctx.part_world_position(ear_1)
    ctx.check(
        "ear_0 and ear_1 on opposite sides (X separation)",
        pos_0 is not None
        and pos_1 is not None
        and pos_0[0] * pos_1[0] < 0
        and abs(pos_0[0]) > 0.05
        and abs(pos_1[0]) > 0.05,
        details=f"ear_0 pos={pos_0}, ear_1 pos={pos_1}",
    )

    # --- each ring anchored to its ear (not floating) ------------------------
    for i in range(2):
        ear = object_model.get_part(f"ear_{i}")
        ring = object_model.get_part(f"ring_{i}")
        ctx.expect_contact(
            ring,
            ear,
            contact_tol=0.008,
            name=f"ring_{i} is anchored to ear_{i} (contact at pivot)",
        )

    # --- barrel base sits at z=0 ----------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "barrel base sits at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-3,
        details=f"body z-min={body_aabb[0][2] if body_aabb else None}",
    )

    # --- body is hollow (thin shell) -----------------------------------------
    inner_r = _outer_radius(BODY_H) - WALL
    ctx.check(
        "body wall is a thin hollow shell",
        WALL < 0.5 * _outer_radius(BODY_H) and inner_r > 0.10,
        details=f"wall={WALL}, inner_r={inner_r:.3f}",
    )

    # --- both hoops present --------------------------------------------------
    for hoop_name, zc in (("upper_hoop", UPPER_HOOP_Z), ("lower_hoop", LOWER_HOOP_Z)):
        hoop = object_model.get_part(hoop_name)
        h_aabb = ctx.part_world_aabb(hoop)
        ctx.check(
            f"{hoop_name} at expected height",
            h_aabb is not None
            and abs((h_aabb[0][2] + h_aabb[1][2]) / 2.0 - zc) < 0.01,
            details=f"{hoop_name} aabb={h_aabb}",
        )

    # --- intentional overlaps ------------------------------------------------
    ctx.allow_overlap(
        "upper_hoop",
        "barrel_body",
        reason="Upper metal hoop intentionally embeds slightly into wood staves.",
    )
    ctx.allow_overlap(
        "lower_hoop",
        "barrel_body",
        reason="Lower metal hoop intentionally embeds slightly into wood staves.",
    )
    for i in range(2):
        ctx.allow_overlap(
            f"ear_{i}",
            "barrel_body",
            reason=f"Ear bracket plate seats against the barrel stave face.",
        )
        ctx.allow_overlap(
            f"ring_{i}",
            f"ear_{i}",
            reason=f"Ring hub passes through the ear fork gap at the pivot.",
        )

    # --- rings swing outward at positive q -----------------------------------
    for i in range(2):
        ring_joint = object_model.get_articulation(f"ear_{i}_to_ring_{i}")
        ring = object_model.get_part(f"ring_{i}")
        # Use AABB center (geometry center) since part origin is the pivot
        rest_aabb = ctx.part_world_aabb(ring)
        with ctx.pose({ring_joint: RING_UPPER}):
            open_aabb = ctx.part_world_aabb(ring)

        if rest_aabb is not None and open_aabb is not None:
            rest_cx = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
            open_cx = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
            sign = 1 if i == 0 else -1
            ctx.check(
                f"ring_{i} swings outward at positive q",
                sign * (open_cx - rest_cx) > 0.005,
                details=f"rest_cx={rest_cx:.4f}, open_cx={open_cx:.4f}",
            )
        else:
            ctx.fail(f"ring_{i} swing check", "could not compute AABB")

    return ctx.report()


object_model = build_object_model()
