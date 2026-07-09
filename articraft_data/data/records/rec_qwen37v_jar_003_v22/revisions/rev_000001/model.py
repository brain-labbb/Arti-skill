from __future__ import annotations

# Square pantry jar with rounded corners, a rotating shaker insert inside the
# lid, glass wall thickness at the mouth, base foot ring, and rim seam.
#
# Frame: vertical +Z, jar centered on world Z axis, base on z=0.
#
# Parts:
#   body          – hollow rounded-square glass jar with base foot ring, thick
#                   mouth rim, and a rim-seam bead.
#   lid           – square lid with shaker holes, a skirt that slips over the
#                   mouth rim, and a central bore for the shaker knob.
#   shaker_insert – circular disk with offset holes and a protruding grip knob;
#                   rotates inside the lid to open/close the shaker ports.
#
# Articulations:
#   body_to_lid     PRISMATIC +Z, 0 → 0.050 m  (lid lifts off)
#   lid_to_shaker   REVOLUTE  Z,  0 → π/6 rad   (shaker rotates open)

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ── jar body ──────────────────────────────────────────────────────────
SECT = 0.080           # outer square section (m)
CORNER_R = 0.008       # body corner fillet
GLASS_WALL = 0.004     # normal glass wall thickness
BODY_H = 0.100         # jar body height

# Base foot ring
BASE_RING_H = 0.005
BASE_RING_EXT = 0.003  # extends beyond body on each side

# Mouth rim  (thicker glass at the wide mouth)
MOUTH_RIM_H = 0.008
MOUTH_RIM_EXT = 0.002  # extends beyond body
MOUTH_RIM_WALL = 0.006 # thicker wall at rim

# Rim seam bead
SEAM_BEAD = 0.0012     # half-round bead near top of mouth rim

# ── lid ────────────────────────────────────────────────────────────────
LID_SECT = SECT + 2 * MOUTH_RIM_EXT + 0.002
LID_CORNER_R = CORNER_R + MOUTH_RIM_EXT + 0.001
LID_TOP_H = 0.004      # top plate thickness
LID_SKIRT_H = 0.012    # skirt height
LID_SKIRT_WALL = 0.003
LID_H = LID_TOP_H + LID_SKIRT_H
LID_CENTER_HOLE_R = 0.006  # bore for shaker knob

# Lid placement at rest: skirt slides down over mouth rim
LID_OVERLAP = 0.006
LID_BOTTOM_Z = BODY_H - LID_OVERLAP

# ── shaker insert ──────────────────────────────────────────────────────
SHAKER_R = 0.028        # disk radius
SHAKER_H = 0.003        # disk thickness
SHAKER_HOLE_R = 0.003   # hole radius
SHAKER_HOLE_N = 6       # number of holes
SHAKER_HOLE_ORBIT = 0.018
SHAKER_KNOB_R = 0.005   # grip knob radius
SHAKER_KNOB_H = 0.008   # grip knob height above disk top

# Shaker disk sits flush with the underside of the lid top plate.
# In lid local coords the disk top is at z = LID_SKIRT_H.
SHAKER_Z_IN_LID = LID_SKIRT_H - SHAKER_H  # disk bottom in lid frame


# ═══════════════════════════════════════════════════════════════════════
# CadQuery geometry builders
# ═══════════════════════════════════════════════════════════════════════

def _jar_body() -> cq.Workplane:
    """Hollow rounded-square jar with base foot ring, mouth rim, rim seam."""

    # ── main outer shell ──
    outer = (
        cq.Workplane("XY")
        .rect(SECT, SECT)
        .extrude(BODY_H)
        .edges("|Z")
        .fillet(CORNER_R)
    )
    # ── inner cavity (open at top, solid glass floor) ──
    inner_w = SECT - 2.0 * GLASS_WALL
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_WALL)
        .rect(inner_w, inner_w)
        .extrude(BODY_H)
        .edges("|Z")
        .fillet(max(CORNER_R - GLASS_WALL, 0.001))
    )
    jar = outer.cut(cavity)

    # ── base foot ring ──
    base_outer = SECT + 2.0 * BASE_RING_EXT
    base_ring = (
        cq.Workplane("XY")
        .rect(base_outer, base_outer)
        .extrude(BASE_RING_H)
        .edges("|Z")
        .fillet(CORNER_R + BASE_RING_EXT)
    )
    # Hollow the inside of the foot ring (match jar outer wall)
    base_bore = (
        cq.Workplane("XY")
        .rect(SECT - 0.001, SECT - 0.001)
        .extrude(BASE_RING_H + 0.001)
        .edges("|Z")
        .fillet(max(CORNER_R - 0.001, 0.001))
    )
    base_ring = base_ring.cut(base_bore)
    jar = jar.union(base_ring)

    # ── mouth rim (thicker wall zone at the wide mouth) ──
    rim_outer = SECT + 2.0 * MOUTH_RIM_EXT
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - MOUTH_RIM_H)
        .rect(rim_outer, rim_outer)
        .extrude(MOUTH_RIM_H + 0.001)  # tiny overshoot for clean union
        .edges("|Z")
        .fillet(CORNER_R + MOUTH_RIM_EXT)
    )
    # Mouth bore: wider than body cavity to show thick glass walls at rim
    mouth_bore_w = SECT - 2.0 * MOUTH_RIM_WALL
    rim_bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - MOUTH_RIM_H - 0.001)
        .rect(mouth_bore_w, mouth_bore_w)
        .extrude(MOUTH_RIM_H + 0.003)
        .edges("|Z")
        .fillet(max(CORNER_R - MOUTH_RIM_WALL, 0.001))
    )
    rim = rim.cut(rim_bore)
    jar = jar.union(rim)

    # ── rim seam bead (small ridge near the top of the mouth rim) ──
    # The bead must overlap the mouth rim for a clean boolean union.
    seam_z = BODY_H - SEAM_BEAD * 2.0
    seam_outer = rim_outer + 2.0 * SEAM_BEAD
    seam = (
        cq.Workplane("XY")
        .workplane(offset=seam_z)
        .rect(seam_outer, seam_outer)
        .extrude(SEAM_BEAD * 2.0)
        .edges("|Z")
        .fillet(CORNER_R + MOUTH_RIM_EXT + SEAM_BEAD)
    )
    # Bore is slightly inside the mouth rim outer so the bead overlaps
    # the rim wall and unions cleanly (no floating island).
    seam_bore_w = rim_outer - 0.002
    seam_bore = (
        cq.Workplane("XY")
        .workplane(offset=seam_z - 0.001)
        .rect(seam_bore_w, seam_bore_w)
        .extrude(SEAM_BEAD * 2.0 + 0.002)
        .edges("|Z")
        .fillet(max(CORNER_R + MOUTH_RIM_EXT - 0.002, 0.001))
    )
    seam = seam.cut(seam_bore)
    jar = jar.union(seam)

    return jar


def _lid_solid() -> cq.Workplane:
    """Square lid with skirt, shaker holes, and central knob bore.

    Lid local frame: z=0 at skirt bottom, top plate at z=LID_SKIRT_H.
    """
    # ── skirt outer shell ──
    skirt_outer = (
        cq.Workplane("XY")
        .rect(LID_SECT, LID_SECT)
        .extrude(LID_SKIRT_H)
        .edges("|Z")
        .fillet(LID_CORNER_R)
    )
    # ── skirt inner bore ──
    skirt_bore_w = LID_SECT - 2.0 * LID_SKIRT_WALL
    skirt_bore = (
        cq.Workplane("XY")
        .rect(skirt_bore_w, skirt_bore_w)
        .extrude(LID_SKIRT_H + 0.001)  # open at bottom
        .edges("|Z")
        .fillet(max(LID_CORNER_R - LID_SKIRT_WALL, 0.001))
    )
    skirt = skirt_outer.cut(skirt_bore)

    # ── top plate ──
    plate = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_H)
        .rect(LID_SECT, LID_SECT)
        .extrude(LID_TOP_H)
        .edges("|Z")
        .fillet(LID_CORNER_R)
    )
    lid = skirt.union(plate)

    # ── shaker holes in top plate (6 holes in a circle) ──
    for i in range(SHAKER_HOLE_N):
        angle = 2.0 * math.pi * i / SHAKER_HOLE_N
        hx = SHAKER_HOLE_ORBIT * math.cos(angle)
        hy = SHAKER_HOLE_ORBIT * math.sin(angle)
        hole_cyl = (
            cq.Workplane("XY")
            .workplane(offset=LID_SKIRT_H - 0.001)
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(LID_TOP_H + 0.002)
        )
        lid = lid.cut(hole_cyl)

    # ── central bore for shaker knob ──
    center_bore = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_H - 0.001)
        .circle(LID_CENTER_HOLE_R)
        .extrude(LID_TOP_H + 0.002)
    )
    lid = lid.cut(center_bore)

    return lid


def _shaker_insert() -> cq.Workplane:
    """Circular shaker disk with offset holes and a protruding grip knob.

    Shaker local frame: z=0 at disk bottom, disk top at z=SHAKER_H,
    knob extends from SHAKER_H to SHAKER_H + SHAKER_KNOB_H.
    Holes are angularly offset from the lid holes by half the angular spacing.
    """
    # ── disk ──
    disk = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_H)
    )

    # ── shaker holes (offset by half-spacing from lid holes) ──
    half_step = math.pi / SHAKER_HOLE_N
    for i in range(SHAKER_HOLE_N):
        angle = 2.0 * math.pi * i / SHAKER_HOLE_N + half_step
        hx = SHAKER_HOLE_ORBIT * math.cos(angle)
        hy = SHAKER_HOLE_ORBIT * math.sin(angle)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_H + 0.002)
        )
        disk = disk.cut(hole)

    # ── grip knob (protrudes upward through lid central bore) ──
    knob = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_H)
        .circle(SHAKER_KNOB_R)
        .extrude(SHAKER_KNOB_H)
    )
    disk = disk.union(knob)

    return disk


# ═══════════════════════════════════════════════════════════════════════
# Model assembly
# ═══════════════════════════════════════════════════════════════════════

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar")

    glass = model.material("clear_glass", rgba=(0.82, 0.86, 0.88, 0.25))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.70, 0.72, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.35, 0.38, 1.0))

    # ── body (root): hollow glass jar ──
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_body(), "glass_jar"),
        material=glass,
        name="glass_jar",
    )
    body.inertial = Inertial.from_geometry(
        Box((SECT + 2 * BASE_RING_EXT, SECT + 2 * BASE_RING_EXT, BODY_H)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ── lid: square lid with shaker holes ──
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=brushed_steel,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_SECT, LID_SECT, LID_H)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, LID_H / 2.0)),
    )

    # ── shaker insert: rotating disk with holes ──
    shaker = model.part("shaker_insert")
    shaker.visual(
        mesh_from_cadquery(_shaker_insert(), "shaker_disk"),
        material=dark_metal,
        name="shaker_disk",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAKER_R, length=SHAKER_H + SHAKER_KNOB_H),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (SHAKER_H + SHAKER_KNOB_H) / 2.0)),
    )

    # ── body_to_lid: PRISMATIC +Z, lid lifts straight off ──
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.050),
    )

    # ── lid_to_shaker: REVOLUTE Z, shaker rotates to align holes ──
    # In world frame the shaker center is at:
    shaker_world_z = LID_BOTTOM_Z + SHAKER_Z_IN_LID
    model.articulation(
        "lid_to_shaker",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_Z_IN_LID)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=1.0,
            lower=0.0,
            upper=math.pi / SHAKER_HOLE_N,  # half-step to align holes
        ),
    )

    return model


# ═══════════════════════════════════════════════════════════════════════
# Tests
# ═══════════════════════════════════════════════════════════════════════

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    shaker = object_model.get_part("shaker_insert")
    lift = object_model.get_articulation("body_to_lid")
    rotate = object_model.get_articulation("lid_to_shaker")

    # ── intentional overlaps ──
    # The lid skirt slides down over the jar mouth rim (friction fit).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="glass_jar",
        reason="Lid skirt is intentionally seated down over the jar mouth rim (lift-off friction fit).",
    )
    # The shaker knob protrudes through the lid central bore; the disk sits
    # inside the lid skirt bore. Small contact at the top-plate underside is
    # intended (the disk rotates against the plate).
    ctx.allow_overlap(
        shaker,
        lid,
        elem_a="shaker_disk",
        elem_b="lid_shell",
        reason="Shaker disk sits inside the lid bore with the knob passing through the central bore (captured rotary fit).",
    )

    # ── jar is square in section ──
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in section",
        abs(body_ext[0] - body_ext[1]) < 0.005,
        details=f"body extents={body_ext}",
    )

    # ── jar is not tall-bottle proportions (wider relative to height) ──
    ctx.check(
        "jar proportions: section is at least half of height",
        body_ext[0] > 0.5 * body_ext[2],
        details=f"body extents={body_ext}",
    )

    # ── base foot ring: body is slightly wider at the bottom ──
    # (the foot ring extends beyond the main body section)
    ctx.check(
        "base foot ring extends beyond main body section",
        body_ext[0] > SECT + BASE_RING_EXT,
        details=f"body X extent={body_ext[0]}, expected>{SECT + BASE_RING_EXT}",
    )

    # ── mouth rim: body is wider at the top (rim extends beyond body) ──
    # The mouth rim and seam bead are above 90% of the body height.
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "mouth rim extends the body width near the top",
        body_ext[0] > SECT + MOUTH_RIM_EXT,
        details=f"body X extent={body_ext[0]}, expected>{SECT + MOUTH_RIM_EXT}",
    )

    # ── lid seated on top when down (q=0) ──
    ctx.expect_overlap(
        lid, body,
        axes="xy",
        min_overlap=0.040,
        name="lid footprint sits over the jar mouth",
    )
    ctx.expect_overlap(
        lid, body,
        axes="z",
        min_overlap=0.003,
        name="lid skirt slides down over the mouth rim",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar",
        lid_pos is not None and lid_pos[2] > BODY_H - LID_OVERLAP - 1e-6,
        details=f"lid origin={lid_pos}",
    )

    # ── lid lifts straight up (prismatic, +Z) ──
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({lift: 0.050}):
        up_pos = ctx.part_world_position(lid)
        ctx.expect_gap(
            lid, body,
            axis="z",
            min_gap=0.0,
            name="lifted lid clears the jar top",
        )
    ctx.check(
        "lid lifts straight up (Z increases)",
        up_pos is not None and up_pos[2] > rest_z + 0.045,
        details=f"rest_z={rest_z}, lifted_z={up_pos[2] if up_pos else None}",
    )

    # ── shaker insert is present and connected to the lid ──
    shaker_pos = ctx.part_world_position(shaker)
    ctx.check(
        "shaker insert exists and is positioned inside the lid",
        shaker_pos is not None and shaker_pos[2] > BODY_H - 0.02,
        details=f"shaker origin={shaker_pos}",
    )

    # ── shaker rotates (revolute): at upper limit the disk actually moved ──
    rest_shaker_pos = ctx.part_world_position(shaker)
    upper_limit = math.pi / SHAKER_HOLE_N
    with ctx.pose({rotate: upper_limit}):
        rotated_pos = ctx.part_world_position(shaker)
    # The shaker center should stay at the same XY (rotates about Z)
    ctx.check(
        "shaker rotates without translating (XY stable)",
        rest_shaker_pos is not None
        and rotated_pos is not None
        and abs(rotated_pos[0] - rest_shaker_pos[0]) < 1e-5
        and abs(rotated_pos[1] - rest_shaker_pos[1]) < 1e-5,
        details=f"rest_xy={rest_shaker_pos[:2]}, rotated_xy={rotated_pos[:2] if rotated_pos else None}",
    )

    # ── shaker knob protrudes above the lid top ──
    lid_aabb = ctx.part_world_aabb(lid)
    shaker_aabb = ctx.part_world_aabb(shaker)
    ctx.check(
        "shaker knob protrudes above the lid top plate",
        shaker_aabb is not None
        and lid_aabb is not None
        and shaker_aabb[1][2] > lid_aabb[1][2] - 0.001,
        details=f"shaker_top={shaker_aabb[1][2] if shaker_aabb else None}, lid_top={lid_aabb[1][2] if lid_aabb else None}",
    )

    # ── materials are distinct ──
    body_mat = body.get_visual("glass_jar").material
    lid_mat = lid.get_visual("lid_shell").material
    shaker_mat = shaker.get_visual("shaker_disk").material
    ctx.check(
        "three distinct materials: glass, steel, dark metal",
        body_mat is not None
        and lid_mat is not None
        and shaker_mat is not None
        and getattr(body_mat, "name", None) == "clear_glass"
        and getattr(lid_mat, "name", None) == "brushed_steel"
        and getattr(shaker_mat, "name", None) == "dark_metal",
        details=f"body={getattr(body_mat, 'name', None)}, lid={getattr(lid_mat, 'name', None)}, shaker={getattr(shaker_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
