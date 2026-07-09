from __future__ import annotations

# Clear plastic square food storage container with a flat snap-on lid.
# Frame: tub centered on the world origin, opening facing +Z. The tub floor is
# near z=0 and the rim mouth is near z=+0.060. The lid sits flat on the rim and
# lifts straight up (PRISMATIC, +Z) off the tub. Everything is tinted-clear.
#
# Articulations:
#   - lid: PRISMATIC, lifts straight up off the tub rim (~0.04 m), no thread/hinge.

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

# ---- overall dimensions (meters) ----
TUB_OUT = 0.130  # outer footprint of the rounded-square tub (X and Y)
TUB_CORNER = 0.022  # corner fillet radius of the rounded square
TUB_H = 0.060  # tub body height (floor to rim)
WALL = 0.0035  # thin wall / floor thickness of the clear shell
RIM_Z = TUB_H  # world z of the rim mouth (tub floor sits at z=0)

LID_PANEL_T = 0.006  # flat lid top panel thickness
LID_SKIRT_H = 0.012  # downward skirt that wraps the tub rim
LID_SKIRT_T = 0.004  # skirt wall thickness
LID_SEAT_DROP = 0.004  # how far the lid skirt overlaps down past the rim (seated)

LIFT_TRAVEL = 0.040  # straight-up travel of the lid off the rim


def _rounded_square(half_side: float, corner: float, height: float) -> cq.Workplane:
    # Solid rounded-square prism, base on z=0, extruded up by `height`.
    return (
        cq.Workplane("XY")
        .rect(2.0 * half_side, 2.0 * half_side)
        .extrude(height)
        .edges("|Z")
        .fillet(corner)
    )


def _tub_solid() -> cq.Workplane:
    # Hollow rounded-square tub, open at the top (+Z). Built by shelling away the
    # top face so the container reads as a real thin-walled clear tub.
    half = TUB_OUT / 2.0
    body = _rounded_square(half, TUB_CORNER, TUB_H)
    # Shell out the top face, leaving WALL-thick walls and floor.
    body = body.faces(">Z").shell(-WALL)
    return body


def _lid_solid() -> cq.Workplane:
    # Flat clear lid: a thin rounded-square top panel with a short downward skirt
    # around its perimeter (a rim lip) that drops over and grips the tub rim.
    # Authored in its own frame with the underside of the skirt at local z=0 and
    # the top panel above it; the joint origin seats it onto the rim.
    half = TUB_OUT / 2.0
    skirt_corner = TUB_CORNER

    # Outer skirt ring: slightly larger than the tub so it wraps the outside.
    skirt_half = half + LID_SKIRT_T
    skirt_outer = _rounded_square(skirt_half, skirt_corner + LID_SKIRT_T, LID_SKIRT_H)
    # Hollow the skirt so it is a ring that slips snugly over the tub rim. The
    # inner face is sized to grip the tub outer wall (a hair smaller than the tub
    # outer footprint) so the seated lid actually contacts the rim, not floats.
    inner_half = half - 0.0008
    skirt_inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .rect(2.0 * inner_half, 2.0 * inner_half)
        .extrude(LID_SKIRT_H + 0.002)
        .edges("|Z")
        .fillet(skirt_corner)
    )
    skirt = skirt_outer.cut(skirt_inner)

    # Flat top panel capping the lid, sitting on top of the skirt.
    panel = _rounded_square(skirt_half, skirt_corner + LID_SKIRT_T, LID_PANEL_T)
    panel = panel.translate((0.0, 0.0, LID_SKIRT_H))

    return skirt.union(panel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_food_container")

    # Tinted-clear plastic: low alpha so the container reads as transparent.
    clear = model.material("clear_plastic", rgba=(0.78, 0.85, 0.88, 0.25))

    # ---- tub (root) ----
    tub = model.part("tub")
    tub.visual(mesh_from_cadquery(_tub_solid(), "tub_shell"), material=clear, name="tub_shell")
    tub.inertial = Inertial.from_geometry(
        Box((TUB_OUT, TUB_OUT, TUB_H)),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, TUB_H / 2.0)),
    )

    # ---- lid: lifts straight up off the rim ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=clear, name="lid_shell")
    lid.inertial = Inertial.from_geometry(
        Box((TUB_OUT, TUB_OUT, LID_SKIRT_H + LID_PANEL_T)),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, (LID_SKIRT_H + LID_PANEL_T) / 2.0)),
    )

    # Seat the lid down so its skirt overlaps the top of the rim by LID_SEAT_DROP.
    # The lid frame underside (local z=0) sits at RIM_Z - LID_SEAT_DROP.
    seat_z = RIM_Z - LID_SEAT_DROP
    model.articulation(
        "tub_to_lid",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, seat_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=LIFT_TRAVEL),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    lid = object_model.get_part("lid")
    lift = object_model.get_articulation("tub_to_lid")

    # The lid skirt intentionally wraps and overlaps the tub rim when seated.
    ctx.allow_overlap(
        lid,
        tub,
        elem_a="lid_shell",
        elem_b="tub_shell",
        reason="The flat lid's skirt lip is intentionally seated over the tub rim when closed.",
    )

    # Container is roughly square in footprint.
    tub_ext = _ext(ctx.part_world_aabb(tub))
    ctx.check(
        "tub footprint is roughly square",
        abs(tub_ext[0] - tub_ext[1]) < 0.01 and tub_ext[0] > 0.10,
        details=f"tub extents={tub_ext}",
    )

    # Container is clear: the clear material alpha is below 1.
    clear = next(m for m in object_model.materials if m.name == "clear_plastic")
    ctx.check(
        "container material is transparent (alpha < 1)",
        clear.rgba is not None and clear.rgba[3] < 1.0,
        details=f"clear rgba={clear.rgba}",
    )

    # Lid covers the tub mouth when seated: lid footprint overlaps the tub in X and Y.
    ctx.expect_overlap(
        lid, tub, axes="xy", min_overlap=0.10, name="lid covers the tub mouth when down"
    )
    # Seated lid skirt overlaps the rim in Z (retained / seated capture).
    ctx.expect_overlap(
        lid, tub, axes="z", min_overlap=0.002, name="lid skirt seated over the rim"
    )

    # The lid lifts STRAIGHT UP off the tub: at max travel it clears the rim and
    # moves only in +Z (footprint stays over the tub).
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({lift: LIFT_TRAVEL}):
        lifted_z = ctx.part_world_position(lid)[2]
        lifted_lid_min = ctx.part_world_aabb(lid)[0]
        tub_max = ctx.part_world_aabb(tub)[1]
        # Footprint stays aligned over the tub (pure vertical motion).
        ctx.expect_overlap(
            lid, tub, axes="xy", min_overlap=0.10, name="lid stays over the tub when lifted"
        )
    ctx.check(
        "lid lifts straight up off the tub",
        lifted_z > rest_z + 0.030,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )
    ctx.check(
        "lifted lid clears the tub rim",
        lifted_lid_min[2] >= tub_max[2] - 0.001,
        details=f"lid_min_z={lifted_lid_min[2]}, tub_max_z={tub_max[2]}",
    )

    return ctx.report()


object_model = build_object_model()
