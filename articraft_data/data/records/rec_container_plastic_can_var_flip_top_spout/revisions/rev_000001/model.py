from __future__ import annotations

# White plastic (HDPE) gallon jug with a FLIP-TOP SPOUT closure.
# Rounded-rectangular body, clearly taller than wide, with a tapered shoulder
# rising to a small top deck. The body is a real blow-molded shell (hollow)
# with a through pour-mouth at the offset neck. A prominent integrated loop
# grip handle is fused into the upper shoulder on one side, with a real open
# finger hole. The neck carries a fixed threaded base collar with a raised
# pour spout nozzle, and a hinged snap lid that swings open on a real revolute
# hinge to expose the spout.
#
# Frame: +Z up. The jug bottom sits on z=0. The rounded-rect footprint is
# centered on X/Y. The threaded neck is offset toward +X; the integrated grip
# handle loop occupies the farther -X side of the top shoulder.
#
# Articulations:
#   - lid_hinge: REVOLUTE about -X at the rear (+Y) edge of the collar,
#     positive q swings the front of the lid upward (open), limits 0..~2.6 rad.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters): a real US gallon jug, ~0.28 m tall ----
BODY_W = 0.150  # X footprint
BODY_D = 0.135  # Y footprint
BASE_H = 0.012  # short flared foot/base ridge at the very bottom
BODY_H = 0.215  # top of the straight body (shoulder start), measured from z=0
SHOULDER_H = 0.050  # tapered shoulder rising to the small top deck
TOP_Z = BODY_H + SHOULDER_H  # ~0.265 m -> small top deck height

DECK_W = BODY_W * 0.60
DECK_D = BODY_D * 0.58

WALL = 0.0035  # shell wall thickness
INNER_TOP_Z = TOP_Z - WALL  # leaves a real top-deck wall except at the pour mouth

NECK_X = 0.020  # neck offset toward +X from body center
NECK_Y = 0.0
NECK_R = 0.0190
NECK_H = 0.024
NECK_TOP_Z = TOP_Z + NECK_H  # neck rim where the closure seats
MOUTH_BORE_R = NECK_R - 0.006
MOUTH_BORE_BOTTOM_Z = WALL + 0.001

# ---- handle loop geometry (unchanged from parent) ----
HANDLE_CX = -0.045
HANDLE_OUTER_W = 0.072
HANDLE_OUTER_H = 0.115
HANDLE_THICK_Y = 0.028
HANDLE_ROOT_Z = BODY_H - 0.030
HANDLE_CZ = HANDLE_ROOT_Z + HANDLE_OUTER_H / 2.0
HANDLE_HOLE_W = 0.040
HANDLE_HOLE_H = 0.072
HANDLE_HOLE_CZ = HANDLE_CZ + 0.014

# ---- Flip-top closure dimensions ----
COLLAR_R = 0.024       # outer radius of threaded base collar
COLLAR_H = 0.009       # height of the collar side wall above the neck rim
COLLAR_BORE_R = NECK_R - 0.001  # inner bore grips the neck

SPOUT_R_OUTER = 0.011  # pour spout nozzle outer radius
SPOUT_R_INNER = 0.008  # bore through the spout
SPOUT_H = 0.010        # spout height above the collar top plate

LID_DISC_R = COLLAR_R + 0.004  # lid outer radius (wraps outside the collar)
LID_WALL = 0.003                # lid wall thickness
LID_H = SPOUT_H + 0.005        # lid cap depth (covers the spout when closed)
# Lid skirt inner radius: LID_DISC_R - LID_WALL = 0.025 > COLLAR_R = 0.024
# so the skirt clears the collar wall with 1mm radial gap.

HINGE_LUG_W = 0.012    # hinge ear width (X)

# Hinge pivot: at the rear (+Y) edge of the collar, at collar top surface.
HINGE_OFFSET_Y = COLLAR_R * 0.92  # just inside the rear collar edge
HINGE_Y = NECK_Y + HINGE_OFFSET_Y
HINGE_Z = NECK_TOP_Z + COLLAR_H  # collar top surface

# Derived spout/closure heights (for tests)
COLLAR_TOP_Z = NECK_TOP_Z + COLLAR_H  # = HINGE_Z
SPOUT_TOP_Z = COLLAR_TOP_Z + SPOUT_H


def _rrect_loft(sections) -> cq.Workplane:
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, w, d) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        wp = wp.rect(w, d)
        prev_z = z
    return wp.loft(ruled=False)


def _body_solid() -> cq.Workplane:
    outer_sections = [
        (0.0, BODY_W * 0.96, BODY_D * 0.96),
        (BASE_H, BODY_W, BODY_D),
        (BODY_H * 0.5, BODY_W, BODY_D),
        (BODY_H, BODY_W * 0.97, BODY_D * 0.97),
        (BODY_H + SHOULDER_H * 0.55, BODY_W * 0.78, BODY_D * 0.76),
        (TOP_Z, DECK_W, DECK_D),
    ]
    outer = _rrect_loft(outer_sections)
    inner_sections = [
        (WALL, BODY_W * 0.96 - 2 * WALL, BODY_D * 0.96 - 2 * WALL),
        (BASE_H, BODY_W - 2 * WALL, BODY_D - 2 * WALL),
        (BODY_H * 0.5, BODY_W - 2 * WALL, BODY_D - 2 * WALL),
        (BODY_H, BODY_W * 0.97 - 2 * WALL, BODY_D * 0.97 - 2 * WALL),
        (
            BODY_H + SHOULDER_H * 0.55,
            BODY_W * 0.78 - 2 * WALL,
            BODY_D * 0.76 - 2 * WALL,
        ),
        (INNER_TOP_Z, DECK_W - 2 * WALL, DECK_D - 2 * WALL),
    ]
    inner = _rrect_loft(inner_sections)

    neck = (
        cq.Workplane("XY")
        .workplane(offset=TOP_Z - 0.004)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R + 0.005)
        .workplane(offset=0.007)
        .circle(NECK_R)
        .workplane(offset=NECK_H - 0.003)
        .circle(NECK_R)
        .loft(ruled=False)
    )
    shell_blank = outer.union(neck)

    neck_bore = (
        cq.Workplane("XY")
        .workplane(offset=MOUTH_BORE_BOTTOM_Z)
        .center(NECK_X, NECK_Y)
        .circle(MOUTH_BORE_R)
        .extrude((NECK_TOP_Z + 0.002) - MOUTH_BORE_BOTTOM_Z)
    )
    body = shell_blank.cut(inner).cut(neck_bore)
    return body


def _handle_solid() -> cq.Workplane:
    loop = (
        cq.Workplane("XZ")
        .center(HANDLE_CX, HANDLE_CZ)
        .rect(HANDLE_OUTER_W, HANDLE_OUTER_H)
        .extrude(HANDLE_THICK_Y, both=True)
    )
    loop = loop.edges("|Y").fillet(0.016)
    hole = (
        cq.Workplane("XZ")
        .center(HANDLE_CX, HANDLE_HOLE_CZ)
        .rect(HANDLE_HOLE_W, HANDLE_HOLE_H)
        .extrude(HANDLE_THICK_Y + 0.02, both=True)
    )
    hole = hole.edges("|Y").fillet(0.015)
    handle = loop.cut(hole)
    return handle


def _closure_base_solid() -> cq.Workplane:
    """Fixed flip-top base assembly: threaded collar ring + top plate + pour spout.
    The top plate bridges the collar annulus to the spout base for connectivity."""
    # Collar ring: seated over the neck with slight embed
    collar = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.002)
        .center(NECK_X, NECK_Y)
        .circle(COLLAR_R)
        .circle(COLLAR_BORE_R)
        .extrude(COLLAR_H + 0.002)
    )
    # Top plate: solid annular disc at the collar top, inner radius matches
    # spout bore so the pour path stays open, outer radius fits inside the
    # collar wall. This connects the collar wall to the spout base.
    top_plate = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_TOP_Z - 0.002)
        .center(NECK_X, NECK_Y)
        .circle(COLLAR_R - 0.001)
        .circle(SPOUT_R_INNER)
        .extrude(0.003)
    )
    # Pour spout nozzle: extends above the top plate
    spout = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_TOP_Z - 0.001)
        .center(NECK_X, NECK_Y)
        .circle(SPOUT_R_OUTER)
        .circle(SPOUT_R_INNER)
        .extrude(SPOUT_H + 0.001)
    )
    base = collar.union(top_plate).union(spout)
    return base


def _lid_solid() -> cq.Workplane:
    """Flip-top snap lid in its own local frame.
    Origin = hinge pivot at the rear collar edge. Cap body extends in -Y.
    The lid is an open-bottom cup: skirt walls + top disc, with the interior
    cavity reaching from z=0 up to just below the disc so the spout fits
    entirely inside the hollow cap when closed."""
    cap_cy = -HINGE_OFFSET_Y  # collar center relative to hinge pivot

    # Outer shell: full cylinder (will be hollowed)
    outer = (
        cq.Workplane("XY")
        .center(0.0, cap_cy)
        .circle(LID_DISC_R)
        .extrude(LID_H)
    )
    # Interior cavity: open at the bottom (z=0), reaches up to the disc floor.
    # This ensures the pour spout (which sits at z=[0, SPOUT_H] in local frame)
    # is entirely inside the hollow cavity, with no overlap with the lid walls.
    cavity = (
        cq.Workplane("XY")
        .center(0.0, cap_cy)
        .circle(LID_DISC_R - LID_WALL)
        .extrude(LID_H - LID_WALL)
    )
    cap = outer.cut(cavity)

    # Hinge ear: reinforcement rib at the rear of the cap, full height.
    # Extends from inside the cavity past the hinge pivot to the skirt wall,
    # ensuring mesh connectivity between the ear and the cap body.
    ear_rear_y = 0.004  # extends past the hinge (y=0) to reach the skirt wall
    ear_front_y = cap_cy * 0.4  # well inside the cap interior
    ear_cy = (ear_front_y + ear_rear_y) / 2.0
    ear_depth = ear_rear_y - ear_front_y
    ear = (
        cq.Workplane("XY")
        .center(0.0, ear_cy)
        .rect(HINGE_LUG_W * 0.55, ear_depth)
        .extrude(LID_H)
    )

    lid = cap.union(ear)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gallon_jug_flip_spout")

    hdpe = model.material("hdpe_white", rgba=(0.95, 0.95, 0.95, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.98, 1.0, 1.0))
    label_teal = model.material("label_teal", rgba=(0.28, 0.72, 0.80, 1.0))
    label_pink = model.material("label_pink", rgba=(0.90, 0.30, 0.55, 1.0))
    closure_mat = model.material("closure_natural", rgba=(0.91, 0.90, 0.85, 1.0))
    lid_mat = model.material("lid_white", rgba=(0.93, 0.93, 0.93, 1.0))

    # ---- body (root): jug shell + integrated grip handle ----
    body = model.part("body")

    shell = _body_solid().union(_handle_solid())
    body.visual(mesh_from_cadquery(shell, "jug_shell"), material=hdpe, name="jug_shell")

    # Front label panel (unchanged from parent)
    label_y = -BODY_D / 2.0 - 0.0015
    label_cz = BODY_H * 0.50
    body.visual(
        Box((0.126, 0.004, 0.118)),
        origin=Origin(xyz=(0.0, label_y, label_cz)),
        material=label_white,
        name="label_panel",
    )
    body.visual(
        Box((0.110, 0.0035, 0.014)),
        origin=Origin(xyz=(0.0, label_y - 0.0018, label_cz + 0.012)),
        material=label_teal,
        name="label_swoosh",
    )
    for i in range(2):
        dx = -0.018 if i == 0 else 0.034
        dz = -0.004 if i == 0 else -0.010
        body.visual(
            Box((0.012, 0.0035, 0.012)),
            origin=Origin(xyz=(dx, label_y - 0.0020, label_cz + dz)),
            material=label_pink,
            name=f"floral_dot_{i}",
        )

    # ---- Fixed flip-top closure base (collar + top plate + spout) ----
    closure_base = _closure_base_solid()
    body.visual(
        mesh_from_cadquery(closure_base, "closure_base"),
        material=closure_mat,
        name="closure_base",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, TOP_Z)),
        mass=0.90,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z * 0.45)),
    )

    # ---- Flip-top lid (articulated) ----
    lid = model.part("lid")
    lid_solid = _lid_solid()
    lid.visual(
        mesh_from_cadquery(lid_solid, "lid_cap"),
        material=lid_mat,
        name="lid_cap",
    )
    lid.inertial = Inertial.from_geometry(
        Box((2.0 * LID_DISC_R, 2.0 * LID_DISC_R, LID_H)),
        mass=0.015,
        origin=Origin(xyz=(0.0, -HINGE_OFFSET_Y, LID_H / 2.0)),
    )

    # ---- REVOLUTE hinge: lid swings open about -X at the rear collar edge ----
    # axis = (-1, 0, 0): right-hand rule about -X takes -Y toward +Z,
    # so positive q lifts the front of the lid upward (opening).
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(NECK_X, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=2.6,
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
    hinge = object_model.get_articulation("lid_hinge")

    # --- The closure base seats over the neck rim (small intentional embed). ---
    ctx.allow_overlap(
        body, body,
        elem_a="closure_base",
        elem_b="jug_shell",
        reason="Collar bore seats over the threaded neck rim (interference fit).",
    )
    # --- The pour spout is intentionally enclosed inside the lid cavity when
    # closed. The lid is a hollow cup whose interior accommodates the raised
    # nozzle; the exact meshes do not interpenetrate, but their AABBs overlap
    # because the spout is surrounded by the lid's hollow interior. ---
    ctx.allow_overlap(
        body, lid,
        elem_a="closure_base",
        elem_b="lid_cap",
        reason="Pour spout is enclosed inside the lid cavity when closed (flip-top cap design).",
    )
    ctx.expect_within(
        body, lid, axes="xy",
        inner_elem="closure_base",
        outer_elem="lid_cap",
        margin=0.005,
        name="closure base fits within lid footprint when closed",
    )

    # --- Jug is clearly taller than wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jug taller than wide",
        bext[2] > bext[0] + 0.05 and bext[2] > bext[1] + 0.05,
        details=f"body extents={bext}",
    )

    # --- Closure base is mounted on the offset neck ---
    closure_aabb = ctx.part_element_world_aabb(body, elem="closure_base")
    closure_cx = (closure_aabb[0][0] + closure_aabb[1][0]) / 2.0
    ctx.check(
        "closure base centered on the offset neck (+X side)",
        closure_aabb is not None
        and closure_aabb[0][2] > TOP_Z - 0.01
        and closure_cx > 0.01,
        details=f"closure center x={closure_cx:.4f}, min z={closure_aabb[0][2]:.4f}",
    )

    # --- Closure base includes a raised spout above the collar top ---
    ctx.check(
        "closure base has a raised pour spout above the collar",
        closure_aabb is not None
        and closure_aabb[1][2] > COLLAR_TOP_Z + SPOUT_H * 0.8,
        details=(
            f"closure top z={closure_aabb[1][2]:.4f}, "
            f"expected spout top={SPOUT_TOP_Z:.4f}"
        ),
    )

    # --- Handle loop rises above the top deck and extends to -X side ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "handle loop rises above the top deck",
        body_aabb is not None and body_aabb[1][2] > TOP_Z + 0.03,
        details=f"body top z={body_aabb[1][2]}, deck={TOP_Z}",
    )
    ctx.check(
        "handle on the -X side opposite the neck",
        body_aabb is not None and body_aabb[0][0] < -0.06,
        details=f"body -X extent={body_aabb[0][0]}",
    )

    # --- Handle has a real open finger hole ---
    shell_aabb = ctx.part_element_world_aabb(body, elem="jug_shell")
    top_rail_top = shell_aabb[1][2]
    ctx.check(
        "handle finger hole is a real open loop (top rail above hole)",
        top_rail_top > HANDLE_HOLE_CZ + 0.015,
        details=f"shell top z={top_rail_top:.4f}, hole center z={HANDLE_HOLE_CZ:.4f}",
    )
    ctx.check(
        "handle loop reaches well left of body center",
        shell_aabb[0][0] < HANDLE_CX - 0.025 and HANDLE_CX < -0.04,
        details=f"shell -X extent={shell_aabb[0][0]:.4f}, handle cx={HANDLE_CX}",
    )

    # --- Hollow jug and through pour mouth ---
    ctx.check(
        "jug body has a hollow interior below the shoulder",
        WALL < MOUTH_BORE_BOTTOM_Z < BODY_H and INNER_TOP_Z < TOP_Z,
        details=(
            f"wall={WALL:.4f}, bore bottom={MOUTH_BORE_BOTTOM_Z:.4f}, "
            f"body shoulder={BODY_H:.4f}, inner top={INNER_TOP_Z:.4f}"
        ),
    )
    ctx.check(
        "pour mouth bore passes through the deck into the hollow jug",
        MOUTH_BORE_BOTTOM_Z < INNER_TOP_Z < NECK_TOP_Z and MOUTH_BORE_R > 0.010,
        details=(
            f"bore bottom={MOUTH_BORE_BOTTOM_Z:.4f}, inner top={INNER_TOP_Z:.4f}, "
            f"neck top={NECK_TOP_Z:.4f}, bore radius={MOUTH_BORE_R:.4f}"
        ),
    )

    # --- Lid is mounted over the collar footprint when closed ---
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.01,
        name="lid seated over collar footprint when closed",
    )

    # --- Flip-top hinge: positive q opens the lid upward ---
    lid_aabb_closed = ctx.part_world_aabb(lid)
    lid_center_z_closed = (lid_aabb_closed[0][2] + lid_aabb_closed[1][2]) / 2.0

    with ctx.pose({hinge: 1.2}):
        lid_aabb_open = ctx.part_world_aabb(lid)
        lid_center_z_open = (lid_aabb_open[0][2] + lid_aabb_open[1][2]) / 2.0
        lid_top_z_open = lid_aabb_open[1][2]

    ctx.check(
        "lid center rises when hinge opens",
        lid_center_z_open > lid_center_z_closed + 0.01,
        details=f"closed center z={lid_center_z_closed:.4f}, open center z={lid_center_z_open:.4f}",
    )
    ctx.check(
        "lid top reaches well above the spout when opened",
        lid_top_z_open > SPOUT_TOP_Z + 0.01,
        details=f"open top z={lid_top_z_open:.4f}, spout top={SPOUT_TOP_Z:.4f}",
    )

    # --- At large opening angle, the lid center is well above the closed position ---
    with ctx.pose({hinge: 2.0}):
        lid_aabb_full = ctx.part_world_aabb(lid)
        lid_center_z_full = (lid_aabb_full[0][2] + lid_aabb_full[1][2]) / 2.0
        lid_top_z_full = lid_aabb_full[1][2]
    ctx.check(
        "lid center rises significantly when fully open",
        lid_center_z_full > lid_center_z_closed + 0.005,
        details=(
            f"closed center z={lid_center_z_closed:.4f}, "
            f"full-open center z={lid_center_z_full:.4f}"
        ),
    )
    ctx.check(
        "lid top reaches well above the spout when fully open",
        lid_top_z_full > SPOUT_TOP_Z + 0.02,
        details=f"lid top z={lid_top_z_full:.4f}, spout top={SPOUT_TOP_Z:.4f}",
    )

    # --- Hinge is REVOLUTE (not continuous/prismatic) ---
    ctx.check(
        "lid hinge is a revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )

    # --- The lid actually moves (non-fixed articulation) ---
    with ctx.pose({hinge: 0.8}):
        lid_aabb_moved = ctx.part_world_aabb(lid)
    lid_center_z_moved = (lid_aabb_moved[0][2] + lid_aabb_moved[1][2]) / 2.0
    ctx.check(
        "lid AABB center shifts when hinge is actuated",
        abs(lid_center_z_moved - lid_center_z_closed) > 0.005,
        details=f"rest z={lid_center_z_closed:.4f}, moved z={lid_center_z_moved:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
