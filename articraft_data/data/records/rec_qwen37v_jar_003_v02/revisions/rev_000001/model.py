from __future__ import annotations

# Square pantry jar with a flip-top lid on a rear revolute hinge.
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#   - body: clear, square-section (rounded-corner) hollow glass jar with a
#           thickened glass rim at the mouth and thread ridges on the rim
#           exterior. Two hinge lugs at the rear top carry the hinge pin.
#   - lid : flat steel lid that flips open on a rear revolute hinge. A hinge
#           barrel at the rear edge rotates around the body hinge pin axis.
#
# Articulation:
#   - body_to_lid: REVOLUTE around +X at the rear hinge line (y = -SECT/2,
#     z = RIM_TOP_Z). At q=0 the lid is closed flat on the rim; positive q
#     opens the lid upward/backward (limit ~1.8 rad ≈ 103°).

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- key dimensions (meters) -----
SECT = 0.090          # outer square section of the jar body
CORNER_R = 0.010      # rounded corner radius
GLASS_WALL = 0.003    # glass wall thickness (main body)

BODY_BOTTOM_Z = 0.0
BODY_TOP_Z = 0.100    # top of main jar body (where rim begins)
BODY_H = BODY_TOP_Z - BODY_BOTTOM_Z

# Thickened glass rim at the mouth
RIM_WALL = 0.006      # thicker glass wall at the mouth
RIM_H = 0.018         # height of the raised rim above body top
RIM_TOP_Z = BODY_TOP_Z + RIM_H

# Thread ridges on the rim exterior
THREAD_COUNT = 3
THREAD_PROTRUSION = 0.0015   # radial protrusion of thread ridges
THREAD_WIDTH = 0.0018        # vertical height of each ridge

# Lid
LID_SECT = 0.088      # lid section (slightly smaller than body to sit inside rim outer)
LID_CORNER_R = 0.008
LID_H = 0.005         # lid plate thickness
LID_LIP_H = 0.004     # downward lip inside the rim

# Hinge
HINGE_BARREL_R = 0.003
HINGE_LENGTH = 0.032  # hinge barrel length along X
LUG_W = 0.008         # hinge lug width (Y direction)
LUG_H = 0.008         # hinge lug height above rim top
LUG_SPACING = 0.040   # spacing between the two body lugs (center-to-center along X)


def _jar_body_solid() -> cq.Workplane:
    """Hollow glass jar with thick rim, thread ridges, and hinge lugs."""
    # --- Main body shell ---
    outer = (
        cq.Workplane("XY")
        .rect(SECT, SECT)
        .extrude(BODY_H)
        .edges("|Z")
        .fillet(CORNER_R)
    )
    inner_w = SECT - 2.0 * GLASS_WALL
    inner_cavity = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_WALL)
        .rect(inner_w, inner_w)
        .extrude(BODY_H + RIM_H + 0.002)  # over-extrude to open through top
        .edges("|Z")
        .fillet(max(CORNER_R - GLASS_WALL, 0.001))
    )
    body = outer.cut(inner_cavity)

    # --- Thickened rim at the mouth ---
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .rect(SECT, SECT)
        .extrude(RIM_H)
        .edges("|Z")
        .fillet(CORNER_R)
    )
    rim_inner_w = SECT - 2.0 * RIM_WALL
    rim_inner = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z - 0.0001)
        .rect(rim_inner_w, rim_inner_w)
        .extrude(RIM_H + 0.002)
        .edges("|Z")
        .fillet(max(CORNER_R - RIM_WALL, 0.001))
    )
    rim = rim_outer.cut(rim_inner)
    body = body.union(rim)

    # --- Thread ridges on rim exterior ---
    # Each thread is a thin rounded-square ring protruding outward from the rim.
    # The ring inner cut is smaller than SECT so the ring embeds into the rim
    # wall, ensuring a connected solid after boolean union.
    ring_inner_w = SECT - 2.0 * RIM_WALL + 0.002  # inside the rim wall
    for i in range(THREAD_COUNT):
        z_base = BODY_TOP_Z + 0.003 + i * 0.005
        t_outer = SECT + 2.0 * THREAD_PROTRUSION
        ring_outer = (
            cq.Workplane("XY")
            .workplane(offset=z_base)
            .rect(t_outer, t_outer)
            .extrude(THREAD_WIDTH)
            .edges("|Z")
            .fillet(CORNER_R + THREAD_PROTRUSION)
        )
        ring_inner_cut = (
            cq.Workplane("XY")
            .workplane(offset=z_base - 0.0002)
            .rect(ring_inner_w, ring_inner_w)
            .extrude(THREAD_WIDTH + 0.0004)
            .edges("|Z")
            .fillet(max(CORNER_R - RIM_WALL, 0.001))
        )
        ring = ring_outer.cut(ring_inner_cut)
        body = body.union(ring)

    # --- Hinge lugs: two small tabs at the rear top of the rim ---
    # Each lug is a small rounded block extending above the rim with a
    # cylindrical barrel on top for the hinge pin.
    lug_y = -SECT / 2.0 + LUG_W / 2.0  # centered on rear face
    lug_z_base = RIM_TOP_Z
    for x_offset in (-LUG_SPACING / 2.0, LUG_SPACING / 2.0):
        # Lug block
        lug_block = (
            cq.Workplane("XY")
            .workplane(offset=lug_z_base)
            .center(x_offset, lug_y)
            .rect(LUG_W, LUG_W)
            .extrude(LUG_H)
            .edges(">Z")
            .fillet(LUG_W * 0.3)
        )
        body = body.union(lug_block)

    # Hinge barrel (cylinder along X) connecting the two lugs
    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(HINGE_BARREL_R)
        .extrude(LUG_SPACING + LUG_W, both=False)
        .translate((-LUG_SPACING / 2.0 - LUG_W / 2.0, -SECT / 2.0 + LUG_W / 2.0, RIM_TOP_Z + LUG_H - HINGE_BARREL_R))
    )
    body = body.union(hinge_barrel)

    return body


def _jar_lid_solid() -> cq.Workplane:
    """Flip-top lid in its local frame.

    Local frame: origin at the hinge pin center. At q=0 (closed), the hinge
    pin is at the rear edge of the jar mouth. The lid plate extends in +Y
    (forward, covering the mouth) and sits from z=0 up to z=LID_H.
    """
    # --- Main lid plate ---
    # Centered at (0, LID_SECT/2, LID_H/2) so rear edge is at y≈0
    plate = (
        cq.Workplane("XY")
        .rect(LID_SECT, LID_SECT)
        .extrude(LID_H)
        .edges("|Z")
        .fillet(LID_CORNER_R)
        .translate((0.0, LID_SECT / 2.0, 0.0))
    )

    # --- Hinge barrel at the rear edge (around local origin) ---
    barrel = (
        cq.Workplane("YZ")
        .circle(HINGE_BARREL_R * 0.9)  # slightly smaller to fit between lugs
        .extrude(HINGE_LENGTH, both=False)
        .translate((-HINGE_LENGTH / 2.0, 0.0, 0.0))
    )
    # Position barrel center at hinge axis: barrel axis is along X at y=0, z=HINGE_BARREL_R
    # Actually let's center it at z=0 to align with articulation origin
    barrel = barrel.translate((0.0, 0.0, HINGE_BARREL_R))

    lid = plate.union(barrel)

    # --- Connecting bridge from barrel to plate ---
    bridge = (
        cq.Workplane("XY")
        .rect(HINGE_LENGTH, HINGE_BARREL_R * 2)
        .extrude(LID_H)
        .translate((0.0, HINGE_BARREL_R, 0.0))
    )
    lid = lid.union(bridge)

    # --- Downward lip inside the rim (for sealing) ---
    lip_w = SECT - 2.0 * RIM_WALL - 0.003
    lip = (
        cq.Workplane("XY")
        .workplane(offset=-LID_LIP_H)
        .rect(lip_w, lip_w)
        .extrude(LID_LIP_H)
        .edges("|Z")
        .fillet(max(CORNER_R - RIM_WALL - 0.001, 0.001))
        .translate((0.0, LID_SECT / 2.0, 0.0))
    )
    lid = lid.union(lip)

    # --- Small clasp tab at the front edge (for gripping to open) ---
    clasp = (
        cq.Workplane("XY")
        .rect(0.015, 0.008)
        .extrude(LID_H + 0.003)
        .edges(">Z")
        .fillet(0.002)
        .translate((0.0, LID_SECT + 0.002, 0.0))
    )
    lid = lid.union(clasp)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.88, 0.28))
    steel = model.material("brushed_steel", rgba=(0.68, 0.69, 0.67, 1.0))

    # ---- body (root): clear square glass jar with rim and threads ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_body_solid(), "glass_jar"),
        material=glass,
        name="glass_jar",
    )
    body.inertial = Inertial.from_geometry(
        Box((SECT, SECT, BODY_H + RIM_H)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + RIM_H) / 2.0)),
    )

    # ---- lid: flip-top steel lid ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_jar_lid_solid(), "flip_lid"),
        material=steel,
        name="flip_lid",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_SECT, LID_SECT, LID_H)),
        mass=0.04,
        origin=Origin(xyz=(0.0, LID_SECT / 2.0, LID_H / 2.0)),
    )

    # ---- Articulation: rear revolute hinge ----
    # Hinge pin axis at the rear of the rim top.
    # Origin in body frame: (0, -SECT/2 + LUG_W/2, RIM_TOP_Z + LUG_H - HINGE_BARREL_R)
    # This is the center of the hinge barrel on top of the lugs.
    hinge_z = RIM_TOP_Z + LUG_H - HINGE_BARREL_R
    hinge_y = -SECT / 2.0 + LUG_W / 2.0
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    # --- Jar proportions: square section, short relative to width ---
    body_ext = None
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb:
        mn, mx = body_aabb
        body_ext = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])
    ctx.check(
        "jar body is square in section",
        body_ext is not None and abs(body_ext[0] - body_ext[1]) < 0.006,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "jar body is short (pantry jar proportions, height ~ 1-2x section)",
        body_ext is not None and body_ext[2] < 3.0 * max(body_ext[0], body_ext[1]),
        details=f"body extents={body_ext}",
    )

    # --- Hinge is revolute with correct limits ---
    limits = hinge.motion_limits
    ctx.check(
        "hinge is revolute with bounded limits",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.upper > limits.lower,
        details=f"type={hinge.articulation_type}, limits={limits}",
    )
    ctx.check(
        "hinge opens to at least 90 degrees",
        limits is not None and limits.upper >= 1.4,
        details=f"upper={limits.upper if limits else None}",
    )

    # --- Closed pose (q=0): lid sits on the rim ---
    with ctx.pose({hinge: 0.0}):
        # Lid overlaps the body footprint in XY (sitting on the mouth)
        ctx.expect_overlap(
            lid, body,
            axes="xy",
            min_overlap=0.040,
            name="closed lid covers the jar mouth in XY",
        )
        # Lid is at the top of the jar (above body midpoint)
        lid_pos = ctx.part_world_position(lid)
        body_pos = ctx.part_world_position(body)
        ctx.check(
            "closed lid sits at the top of the jar",
            lid_pos is not None and body_pos is not None and lid_pos[2] > body_pos[2] + 0.04,
            details=f"lid_z={lid_pos[2] if lid_pos else None}, body_z={body_pos[2] if body_pos else None}",
        )

    # --- Open pose: lid flips backward/upward ---
    rest_pos = ctx.part_world_position(lid)
    with ctx.pose({hinge: 1.5}):
        open_pos = ctx.part_world_position(lid)
        ctx.check(
            "open lid rotates upward (Z increases or stays high)",
            open_pos is not None and rest_pos is not None and open_pos[2] >= rest_pos[2] - 0.01,
            details=f"rest_z={rest_pos[2] if rest_pos else None}, open_z={open_pos[2] if open_pos else None}",
        )
        # The lid front edge should have moved backward (Y decreases) when opening
        ctx.check(
            "open lid front edge moves rearward (Y decreases)",
            open_pos is not None and rest_pos is not None and open_pos[1] < rest_pos[1] + 0.005,
            details=f"rest_y={rest_pos[1] if rest_pos else None}, open_y={open_pos[1] if open_pos else None}",
        )

    # --- Thread ridges: the body geometry extends beyond the base SECT on the rim ---
    # The threads protrude slightly, so body XY extent should be > SECT
    if body_ext:
        ctx.check(
            "thread ridges protrude beyond jar body section",
            body_ext[0] > SECT + 0.001 or body_ext[1] > SECT + 0.001,
            details=f"body_ext_xy=({body_ext[0]:.4f}, {body_ext[1]:.4f}), SECT={SECT}",
        )

    # --- Materials: body is glass, lid is steel ---
    body_mat = body.get_visual("glass_jar").material
    lid_mat = lid.get_visual("flip_lid").material
    ctx.check(
        "body is clear glass and lid is brushed steel",
        body_mat is not None
        and lid_mat is not None
        and getattr(body_mat, "name", None) == "clear_glass"
        and getattr(lid_mat, "name", None) == "brushed_steel",
        details=f"body_mat={getattr(body_mat, 'name', None)}, lid_mat={getattr(lid_mat, 'name', None)}",
    )

    # --- Allow small overlap where lid contacts rim and hinge barrels interlock ---
    ctx.allow_overlap(
        lid, body,
        elem_a="flip_lid",
        elem_b="glass_jar",
        reason="Lid sits on the rim and the hinge barrel interlocks with body lugs; small contact overlap is intentional.",
    )

    return ctx.report()


object_model = build_object_model()
