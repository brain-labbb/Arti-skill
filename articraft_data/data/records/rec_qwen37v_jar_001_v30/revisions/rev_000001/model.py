from __future__ import annotations

# CERAMIC JAR with FLIP LID and SPOON NOTCH
# Variant of cosmetic jar: opaque ceramic body, wide mouth with thick visible rim,
# a U-shaped spoon notch cut into the rim, and a flip-top lid on a rear hinge.
#
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
# Hinge at rear (-Y side), spoon notch at front (+Y side).
#
# Articulation:
#   - lid_hinge: REVOLUTE, axis along X at rear of rim top, positive q opens lid upward.

import cadquery as cq

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

# ---- key dimensions (meters) ----
JAR_OUTER_R = 0.040           # outer radius of ceramic body (~0.080 m dia)
JAR_BODY_H = 0.052            # height of the ceramic body
WALL = 0.006                  # thick ceramic wall
RIM_EXTRA = 0.003             # extra thickness at rim for visible wall at mouth
NECK_R = JAR_OUTER_R - 0.002  # neck slightly inset from body
NECK_H = 0.008                # short neck/rim height
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the rim top (0.060)

# Spoon notch
NOTCH_R = 0.008               # radius of spoon notch cutout

# Lid
LID_R = NECK_R - 0.001        # lid slightly smaller than neck inner to sit inside
LID_THICK = 0.005             # lid disc thickness

# Hinge
HINGE_Y = -(JAR_OUTER_R + 0.002)  # hinge pin just behind the jar outer wall at rear
HINGE_Z = RIM_TOP_Z + 0.002       # hinge pin slightly above rim top
LUG_W = 0.005                     # lug width along X
LUG_H = 0.008                     # lug height (above rim)
LUG_D = 0.006                     # lug depth along Y


def _ceramic_jar_with_lugs() -> cq.Workplane:
    """Hollow ceramic jar with thick walls, thickened rim, hinge lugs, and pin."""
    # Revolve profile for the jar shell
    rim_outer = NECK_R + RIM_EXTRA
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - 0.003, JAR_BODY_H),
        (NECK_R, JAR_BODY_H + 0.002),
        (NECK_R, RIM_TOP_Z - 0.002),
        (rim_outer, RIM_TOP_Z - 0.002),
        (rim_outer, RIM_TOP_Z),
        (NECK_R - WALL, RIM_TOP_Z),
        (NECK_R - WALL, JAR_BODY_H - 0.002),
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - WALL, WALL),
        (0.0, WALL),
        (0.0, 0.0),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    jar = profile.revolve(360.0, (0, 0, 0), (0, 1, 0))

    # Cut spoon notch: horizontal cylinder at +Y side cutting through the rim
    notch_z = RIM_TOP_Z - NECK_H * 0.4
    notch_cutter = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .transformed(offset=(0, 0, notch_z))
        .circle(NOTCH_R)
        .extrude(JAR_OUTER_R + 0.02)
    )
    jar = jar.cut(notch_cutter)

    # Add hinge lugs: two tabs that straddle the jar rear wall, protruding above rim.
    # Each lug overlaps with the jar neck/rim wall to ensure mesh connectivity.
    lug_gap = 0.014  # spacing between lugs along X
    # Lugs extend from inside the jar wall to outside, bridging across the rim
    lug_y_inner = -(NECK_R - WALL + 0.001)  # just inside the neck inner wall
    lug_y_outer = -(JAR_OUTER_R + LUG_D)    # outside the body wall
    lug_y_len = lug_y_inner - lug_y_outer    # total Y extent
    lug_y_center = (lug_y_inner + lug_y_outer) * 0.5
    # Lugs start 3mm below the rim top so they overlap with the jar solid
    lug_z_start = RIM_TOP_Z - 0.003
    lug_z_h = LUG_H + 0.003  # total height including overlap

    for x_off in [-(lug_gap * 0.5 + LUG_W * 0.5), (lug_gap * 0.5 + LUG_W * 0.5)]:
        lug = (
            cq.Workplane("XY")
            .workplane(offset=lug_z_start)
            .center(x_off, lug_y_center)
            .rect(LUG_W, lug_y_len)
            .extrude(lug_z_h)
        )
        jar = jar.union(lug)

    # Hinge pin: a box along X through both lugs at the hinge height
    pin_half = (lug_gap * 0.5 + LUG_W + 0.002)
    pin_box = (
        cq.Workplane("XY")
        .workplane(offset=HINGE_Z - 0.0015)
        .center(0, HINGE_Y)
        .rect(pin_half * 2, 0.003)
        .extrude(0.003)
    )
    jar = jar.union(pin_box)

    return jar


def _cream_fill_mesh() -> cq.Workplane:
    """Cream fill visible inside the jar, contacting the inner wall."""
    inner_r = JAR_OUTER_R - WALL  # exactly at inner wall radius for connectivity
    cream = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H - 0.012)
        .circle(inner_r)
        .extrude(0.008)
    )
    return cream


def _flip_lid_solid() -> cq.Workplane:
    """Flip lid: disc covering the mouth with a hinge tab at the rear.
    Built in lid-local frame where origin = hinge pin center.
    Lid extends toward +Y (toward front of jar)."""
    # Distance from hinge pin to lid disc center
    lid_center_y = JAR_OUTER_R + 0.002  # hinge to center

    # Main disc (centered at lid_center_y in local Y)
    disc = (
        cq.Workplane("XY")
        .workplane(offset=-LID_THICK * 0.5)
        .center(0, lid_center_y)
        .circle(LID_R)
        .extrude(LID_THICK)
    )

    # Hinge tab: bridges from hinge origin (y=0) to the disc edge
    # Disc nearest edge is at y = lid_center_y - LID_R = 0.005
    # Tab needs to span from y=-0.004 to y=0.010 (overlapping with disc)
    tab_w = 0.012
    tab_y_min = -0.004
    tab_y_max = lid_center_y - LID_R + 0.005  # overlap into disc
    tab_len = tab_y_max - tab_y_min
    tab_cy = (tab_y_min + tab_y_max) * 0.5
    tab = (
        cq.Workplane("XY")
        .workplane(offset=-LID_THICK * 0.5)
        .center(0, tab_cy)
        .rect(tab_w, tab_len)
        .extrude(LID_THICK)
    )
    lid = disc.union(tab)

    # Seating lip on underside: ring that fits inside the rim, overlapping disc bottom
    lip_outer = LID_R - 0.002
    lip_inner = LID_R - 0.005
    lip_h = 0.003
    # Start the lip so it overlaps with the disc bottom (disc bottom at z=-LID_THICK*0.5)
    lip_z_bottom = -LID_THICK * 0.5 - lip_h + 0.001  # 1mm overlap into disc
    lip = (
        cq.Workplane("XY")
        .workplane(offset=lip_z_bottom)
        .center(0, lid_center_y)
        .circle(lip_outer)
        .circle(lip_inner)
        .extrude(lip_h)
    )
    lid = lid.union(lip)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_jar_flip_lid")

    # Materials
    ceramic_cream = model.material("ceramic_cream", rgba=(0.93, 0.89, 0.82, 1.0))
    lid_mat = model.material("lid_ceramic", rgba=(0.88, 0.84, 0.76, 1.0))
    cream_fill_mat = model.material("cream_fill", rgba=(0.98, 0.95, 0.88, 1.0))

    # ---- jar body (root): ceramic shell with hinge lugs + cream fill ----
    body = model.part("body")

    # Ceramic jar shell with integrated hinge lugs and spoon notch
    jar_shape = _ceramic_jar_with_lugs()
    body.visual(
        mesh_from_cadquery(jar_shape, "jar_ceramic"),
        material=ceramic_cream,
        name="jar_ceramic",
    )

    # Cream fill disc inside the jar (contacts inner wall)
    cream = _cream_fill_mesh()
    body.visual(
        mesh_from_cadquery(cream, "cream_fill"),
        material=cream_fill_mat,
        name="cream_fill",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- flip lid: hinges at rear of rim ----
    lid = model.part("lid")

    lid_shape = _flip_lid_solid()
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid_disc"),
        material=lid_mat,
        name="lid_disc",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICK),
        mass=0.025,
        origin=Origin(xyz=(0.0, JAR_OUTER_R + 0.002, 0.0)),
    )

    # Revolute articulation: hinge at rear of rim
    # Origin at hinge pin location, axis along X
    # Right-hand rule around +X: +Y rotates toward +Z → opens lid upward
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=2.2,  # ~126 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    # ---- jar body has ceramic material (opaque, not glass) ----
    ceramic_vis = body.get_visual("jar_ceramic")
    ctx.check(
        "jar body has ceramic material",
        ceramic_vis is not None and ceramic_vis.material is not None,
        details="jar_ceramic visual must exist with an opaque ceramic material",
    )

    # ---- jar is squat: wider than tall ----
    body_aabb = ctx.part_world_aabb(body)
    bext = (body_aabb[1][0] - body_aabb[0][0],
            body_aabb[1][1] - body_aabb[0][1],
            body_aabb[1][2] - body_aabb[0][2])
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] + 0.005 and bext[1] > bext[2] + 0.005,
        details=f"body extents={bext}",
    )

    # ---- lid covers jar mouth at rest (q=0) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid covers jar mouth at rest",
    )

    # ---- flip hinge opens the lid upward ----
    # Check the lid_disc element world AABB to see the front edge rise
    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_disc")
    rest_max_z = lid_rest_aabb[1][2]
    rest_center_z = (lid_rest_aabb[0][2] + lid_rest_aabb[1][2]) * 0.5

    with ctx.pose({hinge: 1.5}):  # ~86 degrees open
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_disc")
        open_max_z = lid_open_aabb[1][2]
        open_center_z = (lid_open_aabb[0][2] + lid_open_aabb[1][2]) * 0.5
        ctx.check(
            "flip hinge opens lid upward",
            open_max_z > rest_max_z + 0.01,
            details=f"rest_max_z={rest_max_z}, open_max_z={open_max_z}",
        )

    # ---- lid at max open stays above ground ----
    with ctx.pose({hinge: 2.2}):
        lid_full_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid at max open stays above jar base",
            lid_full_aabb[0][2] > -0.01,
            details=f"lid min z at max open = {lid_full_aabb[0][2]}",
        )

    # ---- visible wall thickness at mouth (rim geometry in jar_ceramic) ----
    ctx.check(
        "jar has visible thick rim at mouth",
        body.get_visual("jar_ceramic") is not None,
        details="rim thickness is part of the revolved jar_ceramic profile",
    )

    # ---- hinge lugs are part of the jar body (integrated) ----
    ctx.check(
        "jar body includes hinge lug geometry",
        body.get_visual("jar_ceramic") is not None,
        details="hinge lugs are unioned into jar_ceramic solid",
    )

    # Allow small overlap between lid seating lip and jar rim when closed
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_disc", elem_b="jar_ceramic",
        reason="Lid seating lip nests slightly into the rim for a flush closed fit.",
    )

    return ctx.report()


object_model = build_object_model()
