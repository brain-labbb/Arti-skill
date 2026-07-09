from __future__ import annotations

# Clear round plastic deli tub with a flat snap-on lid.
# Frame: tub axis vertical along +Z, centerline at x=y=0, tub base at z=0.
# Tapered round hollow shell (wider at the mouth) with a rolled rim flange at
# the top; the lid is a flat clear disc with a downturned rim lip that seats
# over the tub mouth.
# Articulations:
#   - lid: PRISMATIC, lifts straight up off the tub rim (+Z), no thread/hinge.

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
TUB_BASE_R = 0.047  # bottom radius (slightly narrower)
TUB_MOUTH_R = 0.058  # mouth radius (wider -> tapered walls)
TUB_HEIGHT = 0.058  # wall height (tub is wider than tall)
WALL_T = 0.0014  # thin transparent wall
RIM_R = 0.062  # outer radius of the rolled rim flange
RIM_Z = TUB_HEIGHT  # rim sits at the mouth

LID_R = RIM_R + 0.0015  # lid disc slightly overhangs the rim
LID_TOP_T = 0.0016  # flat lid panel thickness
LID_LIP_DROP = 0.010  # downturned skirt that grips the rim
LID_LIFT = 0.040  # straight-up travel to clear the rim


def _tub_solid() -> cq.Workplane:
    # Tapered outer shell minus a slightly smaller inner cavity so the tub is a
    # real open-topped hollow vessel, plus a rolled rim flange ring at the mouth.
    outer = (
        cq.Workplane("XY")
        .circle(TUB_BASE_R)
        .workplane(offset=TUB_HEIGHT)
        .circle(TUB_MOUTH_R)
        .loft(ruled=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL_T)  # leave a base floor
        .circle(TUB_BASE_R - WALL_T)
        .workplane(offset=TUB_HEIGHT)  # poke past the mouth so the top is open
        .circle(TUB_MOUTH_R - WALL_T)
        .loft(ruled=True)
    )
    shell = outer.cut(inner)

    # Rolled rim flange: a thin torus-like ring around the mouth lip.
    rim = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z)
        .circle(RIM_R)
        .circle(TUB_MOUTH_R - WALL_T)
        .extrude(-0.004)
    )
    return shell.union(rim)


def _lid_solid() -> cq.Workplane:
    # Flat clear lid panel with a short downturned rim lip (skirt) that snaps
    # over the tub rim flange. Built about z=0 at the lip bottom.
    panel = (
        cq.Workplane("XY")
        .workplane(offset=LID_LIP_DROP)
        .circle(LID_R)
        .extrude(LID_TOP_T)
    )
    # Downturned skirt: a thin annular wall hanging from the panel edge.
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .circle(LID_R - 0.0030)
        .extrude(LID_LIP_DROP + LID_TOP_T)
    )
    # A subtle recessed center step on the top (stackable deli lid look).
    step = (
        cq.Workplane("XY")
        .workplane(offset=LID_LIP_DROP + LID_TOP_T)
        .circle(LID_R - 0.006)
        .extrude(0.0016)
    )
    return panel.union(skirt).union(step)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deli_tub")

    # Tinted-clear PET plastic: low alpha so it reads transparent.
    clear = model.material("clear_plastic", rgba=(0.82, 0.86, 0.88, 0.25))

    # ---- tub (root) ----
    tub = model.part("tub")
    tub.visual(mesh_from_cadquery(_tub_solid(), "tub_shell"), material=clear, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((2 * RIM_R, 2 * RIM_R, TUB_HEIGHT)),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, TUB_HEIGHT / 2.0)),
    )

    # ---- lid: lifts straight up off the rim (prismatic, +Z) ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=clear, name="lid_shell")
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_R, 2 * LID_R, LID_LIP_DROP + LID_TOP_T)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (LID_LIP_DROP + LID_TOP_T) / 2.0)),
    )
    # Seated: lid lip bottom sits just below the rim top so the skirt grips it.
    model.articulation(
        "tub_to_lid",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, RIM_Z - 0.0035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.1, lower=0.0, upper=LID_LIFT),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("tub_to_lid")

    # Tub is round (x/y extents nearly equal) and wider than tall.
    tub_ext = _ext(ctx.part_world_aabb(tub))
    ctx.check(
        "tub is round",
        abs(tub_ext[0] - tub_ext[1]) < 0.004,
        details=f"tub extents={tub_ext}",
    )
    ctx.check(
        "tub is wider than tall",
        tub_ext[0] > tub_ext[2] + 0.02 and tub_ext[1] > tub_ext[2] + 0.02,
        details=f"tub extents={tub_ext}",
    )

    # Tub is clear: the only material is low-alpha tinted plastic.
    clear = next(m for m in object_model.materials if m.name == "clear_plastic")
    ctx.check(
        "tub material is transparent (alpha < 1)",
        clear.rgba[3] < 1.0,
        details=f"alpha={clear.rgba[3]}",
    )

    # Lid covers the mouth when down: its footprint spans the tub mouth.
    ctx.expect_within(
        tub,
        lid,
        axes="xy",
        margin=0.001,
        name="lid covers the tub mouth footprint",
    )

    # Lid seats on the tub rim (small intentional skirt-over-rim grip).
    ctx.allow_overlap(
        lid,
        tub,
        elem_a="lid_shell",
        elem_b="tub_shell",
        reason="The lid skirt intentionally slips over the tub rim flange to read as snapped on.",
    )
    ctx.expect_contact(lid, tub, name="lid rests on the tub rim")

    # Lid lifts straight up off the tub and fully clears the rim.
    rest_z = ctx.part_world_position(lid)[2]
    rest_min_z = ctx.part_world_aabb(lid)[0][2]
    with ctx.pose({lid_joint: LID_LIFT}):
        lifted_z = ctx.part_world_position(lid)[2]
        lifted_min_z = ctx.part_world_aabb(lid)[0][2]
        tub_max_z = ctx.part_world_aabb(tub)[1][2]
    ctx.check(
        "lid lifts straight up",
        lifted_z > rest_z + 0.035,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )
    ctx.check(
        "lifted lid clears the tub rim",
        lifted_min_z >= tub_max_z - 0.001,
        details=f"lifted lid min_z={lifted_min_z}, tub max_z={tub_max_z}, rest min_z={rest_min_z}",
    )

    return ctx.report()


object_model = build_object_model()
