from __future__ import annotations

# Open-slatted wooden crate box forked from the walnut keepsake box.
# The solid side walls are replaced with horizontal wooden slats separated
# by visible gaps, giving the classic open-crate look.  Corner posts in
# lighter maple carry the structure; walnut slats span between them.
# The flat lid and rear hinge remain unchanged.
#
# Frame:
#   +X = box width (0.24 m), +Y = box depth (0.15 m), +Z = up (height 0.10 m).
#   Rear of the box is +Y; front (where the lid free edge sits) is -Y.
#   Everything is centered on x=0, y=0; the base sits on z=0 and grows up.
#
# Articulation:
#   - lid: REVOLUTE about the rear-top hinge line (axis = -X). The closed lid
#     geometry extends from the rear hinge toward -Y, so positive q lifts the
#     front edge up and swings it back over the rear (~0 to 100 degrees).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)

# ---- Box dimensions (identical to parent) ----
W = 0.240          # outer width  (X)
D = 0.150          # outer depth  (Y)
WALL = 0.012       # slat thickness (wall depth)
BASE_T = 0.012     # floor panel thickness
BODY_H = 0.090     # outer height of the box base (without lid)
INNER_H = BODY_H - BASE_T  # usable wall height above the floor

LID_T = 0.014      # lid slab thickness
LID_PANEL_INSET = 0.010

# Hinge sits at the rear top edge of the box base.
HINGE_Y = D / 2.0 - WALL / 2.0
HINGE_Z = BODY_H

# ---- Crate-specific dimensions ----
POST_SIZE = 0.016  # corner-post cross-section (square)
SLAT_H = 0.012     # slat board height
SLAT_GAP = 0.010   # vertical gap between consecutive slats
N_SLATS = 4        # slats per wall  (4 * 0.012 + 3 * 0.010 = 0.078 = INNER_H)


# ---------------------------------------------------------------------------
# Shared geometry helper
# ---------------------------------------------------------------------------

def _slat_board(length: float, height: float, thickness: float) -> cq.Workplane:
    """One horizontal slat board.

    Long axis along X, thin along Y, starting at z = 0.
    Centered on x = 0, y = 0 in the XY plane.
    """
    return (
        cq.Workplane("XY")
        .box(length, thickness, height, centered=(True, True, False))
    )


# ---------------------------------------------------------------------------
# Static solids
# ---------------------------------------------------------------------------

def _floor_panel_solid() -> cq.Workplane:
    """Solid floor panel spanning the full footprint."""
    return (
        cq.Workplane("XY")
        .box(W, D, BASE_T, centered=(True, True, False))
    )


def _corner_posts_solid() -> cq.Workplane:
    """Four square corner posts carrying the slatted walls."""
    half_w = W / 2.0
    half_d = D / 2.0
    post_centers = [
        ( half_w - POST_SIZE / 2.0,  half_d - POST_SIZE / 2.0),
        (-half_w + POST_SIZE / 2.0,  half_d - POST_SIZE / 2.0),
        ( half_w - POST_SIZE / 2.0, -half_d + POST_SIZE / 2.0),
        (-half_w + POST_SIZE / 2.0, -half_d + POST_SIZE / 2.0),
    ]
    result = None
    for px, py in post_centers:
        post = (
            cq.Workplane("XY")
            .center(px, py)
            .box(POST_SIZE, POST_SIZE, BODY_H, centered=(True, True, False))
        )
        result = post if result is None else result.union(post)
    return result


def _lid_solid() -> cq.Workplane:
    """Flat lid slab (local frame: hinge line at origin, extends toward -Y)."""
    return (
        cq.Workplane("XY")
        .center(0.0, -D / 2.0)
        .box(W, D, LID_T, centered=(True, True, False))
        .translate((0.0, 0.0, -LID_T))
    )


def _lid_panel_solid() -> cq.Workplane:
    """Recessed lighter inner panel on the underside of the lid."""
    pw = W - 2 * WALL - 2 * LID_PANEL_INSET
    pd = D - 2 * WALL - 2 * LID_PANEL_INSET
    return (
        cq.Workplane("XY")
        .center(0.0, -D / 2.0)
        .box(pw, pd, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -LID_T - 0.004))
    )


# ---------------------------------------------------------------------------
# Model build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slatted_crate_box")

    walnut = model.material("walnut", rgba=(0.36, 0.22, 0.13, 1.0))
    maple  = model.material("maple_accent", rgba=(0.82, 0.66, 0.42, 1.0))
    brass  = model.material("brass", rgba=(0.74, 0.58, 0.26, 1.0))

    # ---- box base (root) ----
    body = model.part("box_base")

    # Crate structure: floor panel and corner posts as separate visuals (maple)
    body.visual(
        mesh_from_cadquery(_floor_panel_solid(), "floor_panel"),
        material=maple,
        name="floor_panel",
    )
    body.visual(
        mesh_from_cadquery(_corner_posts_solid(), "corner_posts"),
        material=maple,
        name="corner_posts",
    )

    # Slatted walls: 4 walls × N_SLATS slats, emitted by a for-i-in-range loop
    half_w = W / 2.0
    half_d = D / 2.0
    slat_pitch = SLAT_H + SLAT_GAP

    # Front/back slats span between corner posts in X.
    fb_slat_len = W - 2.0 * POST_SIZE
    # Left/right slats span between corner posts in Y.
    lr_slat_len = D - 2.0 * POST_SIZE

    # (wall_name, slat_length, center_x, center_y, rotate_90_about_Z)
    walls = [
        ("front", fb_slat_len,  0.0,             -half_d + WALL / 2.0, False),
        ("back",  fb_slat_len,  0.0,              half_d - WALL / 2.0, False),
        ("left",  lr_slat_len, -half_w + WALL / 2.0, 0.0,             True),
        ("right", lr_slat_len,  half_w - WALL / 2.0, 0.0,             True),
    ]

    for wall_name, length, cx, cy, rotate in walls:
        for i in range(N_SLATS):
            z_base = BASE_T + i * slat_pitch
            slat = _slat_board(length, SLAT_H, WALL)
            if rotate:
                slat = slat.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)
            slat = slat.translate((cx, cy, z_base))
            body.visual(
                mesh_from_cadquery(slat, f"{wall_name}_slat_{i}"),
                material=walnut,
                name=f"{wall_name}_slat_{i}",
            )

    # Brass hinge knuckles on the rear top edge of the base (same as parent).
    for i, kx in enumerate((-0.055, 0.055)):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=kx - 0.010)
            .center(HINGE_Y, HINGE_Z)
            .circle(0.006)
            .extrude(0.020)
        )
        body.visual(
            mesh_from_cadquery(knuckle, f"base_knuckle_{i}"),
            material=brass,
            name=f"base_knuckle_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid (unchanged from parent) ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_slab"),
        material=walnut,
        name="lid_slab",
    )
    lid.visual(
        mesh_from_cadquery(_lid_panel_solid(), "lid_panel"),
        material=maple,
        name="lid_panel",
    )
    # Lid-side hinge knuckles, interleaved with the base knuckles.
    for i, kx in enumerate((-0.020, 0.085)):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=kx - 0.010)
            .center(0.0, 0.0)
            .circle(0.006)
            .extrude(0.020)
        )
        lid.visual(
            mesh_from_cadquery(knuckle, f"lid_knuckle_{i}"),
            material=brass,
            name=f"lid_knuckle_{i}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((W, D, LID_T)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -D / 2.0, -LID_T / 2.0)),
    )

    # Hinge: axis along the rear top edge (same as parent).
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> "TestReport":
    from sdk import TestContext

    ctx = TestContext(object_model)

    body  = object_model.get_part("box_base")
    lid   = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    top_slat_idx = N_SLATS - 1  # index of the topmost slat on each wall

    # ---- Overlap allowances ----

    # Interleaved hinge knuckles share the rear hinge pin line.
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_knuckle_0", elem_b="base_knuckle_0",
        reason="Interleaved hinge knuckles share the rear hinge pin line.",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_knuckle_1", elem_b="base_knuckle_1",
        reason="Interleaved hinge knuckles share the rear hinge pin line.",
    )

    # Closed lid slab seats onto the corner posts at the top rim.
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_slab", elem_b="corner_posts",
        reason="Closed lid slab seats flush onto the corner-post top rim.",
    )

    # Closed lid slab rests on each wall's top slat.
    for wall in ("front", "back", "left", "right"):
        ctx.allow_overlap(
            lid, body,
            elem_a="lid_slab", elem_b=f"{wall}_slat_{top_slat_idx}",
            reason="Closed lid slab seats onto the top slat of each wall.",
        )

    # Lid hinge knuckles mount at the rear, embedding the corner posts and top back slat.
    for i in (0, 1):
        ctx.allow_overlap(
            lid, body,
            elem_a=f"lid_knuckle_{i}", elem_b="corner_posts",
            reason="Lid hinge knuckles are mounted onto the rear corner-post top.",
        )
        ctx.allow_overlap(
            lid, body,
            elem_a=f"lid_knuckle_{i}", elem_b=f"back_slat_{top_slat_idx}",
            reason="Lid hinge knuckles sit at the rear top-slat line.",
        )

    # Closed lid slab covers the base hinge knuckles.
    for i in (0, 1):
        ctx.allow_overlap(
            lid, body,
            elem_a="lid_slab", elem_b=f"base_knuckle_{i}",
            reason="Closed lid slab rests over the rear hinge knuckles.",
        )

    # ---- Prompt-specific checks: slatted crate walls ----

    # 1) All slat visuals exist on the box_base part.
    expected_slat_names = []
    for wall in ("front", "back", "left", "right"):
        for i in range(N_SLATS):
            expected_slat_names.append(f"{wall}_slat_{i}")

    all_present = all(
        body.get_visual(n) is not None for n in expected_slat_names
    )
    ctx.check(
        "all slatted wall visuals exist",
        all_present,
        details=f"expected {len(expected_slat_names)} slat visuals across 4 walls",
    )

    # 2) Visible gaps between consecutive slats on each wall (open crate look).
    for wall in ("front", "back", "left", "right"):
        for i in range(N_SLATS - 1):
            lower_name = f"{wall}_slat_{i}"
            upper_name = f"{wall}_slat_{i + 1}"
            ctx.expect_gap(
                body, body,
                axis="z",
                positive_elem=upper_name,
                negative_elem=lower_name,
                min_gap=SLAT_GAP * 0.5,
                name=f"gap between {lower_name} and {upper_name}",
            )

    # 3) Bottom slats sit on the floor panel (no large gap, no deep embed).
    for wall in ("front", "back", "left", "right"):
        ctx.expect_gap(
            body, body,
            axis="z",
            positive_elem=f"{wall}_slat_0",
            negative_elem="floor_panel",
            min_gap=-0.001,
            max_gap=0.003,
            name=f"bottom {wall} slat sits on the floor panel",
        )

    # ---- Inherited parent checks: lid, hinge, footprint ----

    # 4) Box base spans full footprint.
    base_aabb = ctx.part_world_aabb(body)
    bmn, bmx = base_aabb
    ctx.check(
        "box base spans full footprint",
        (bmx[0] - bmn[0]) > 0.22 and (bmx[1] - bmn[1]) > 0.13 and (bmx[2] - bmn[2]) > 0.08,
        details=f"base extents={(bmx[0]-bmn[0], bmx[1]-bmn[1], bmx[2]-bmn[2])}",
    )

    # 5) Closed lid covers the box footprint and sits at the top.
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.10,
        name="closed lid covers the box footprint",
    )
    lid_aabb = ctx.part_world_aabb(lid)
    lmn, lmx = lid_aabb
    ctx.check(
        "closed lid sits at the box top",
        lmx[2] >= bmx[2] - 0.002,
        details=f"lid_top_z={lmx[2]}, base_top_z={bmx[2]}",
    )

    # 6) Hinge at the rear top edge.
    ctx.check(
        "hinge at rear top edge",
        HINGE_Y > 0.0 and abs(HINGE_Z - bmx[2]) < 0.01,
        details=f"hinge_y={HINGE_Y}, hinge_z={HINGE_Z}, base_top={bmx[2]}",
    )

    # 7) Opening the lid: front edge lifts and swings rearward.
    front_y_closed = lmn[1]
    top_z_closed = lmx[2]
    with ctx.pose({hinge: math.radians(100.0)}):
        op_mn, op_mx = ctx.part_world_aabb(lid)
        front_y_open = op_mn[1]
        top_z_open = op_mx[2]
    ctx.check(
        "lid front edge lifts when opened",
        top_z_open > top_z_closed + 0.05,
        details=f"top_z closed={top_z_closed}, open={top_z_open}",
    )
    ctx.check(
        "lid front edge swings rearward when opened",
        front_y_open > front_y_closed + 0.02,
        details=f"front_y closed={front_y_closed}, open={front_y_open}",
    )

    return ctx.report()


object_model = build_object_model()
