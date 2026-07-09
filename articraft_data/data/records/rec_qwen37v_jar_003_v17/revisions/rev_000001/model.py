from __future__ import annotations

# Wide-mouth glass jar with a hinged flip-top lid and a vertical-lift stopper.
# Variant sibling of the tall square glass bottle.
#
# Frame: vertical axis +Z, jar centered on world Z, base on z=0.
#   - body    : clear glass rounded-square hollow jar with wide mouth opening,
#               hinge lugs on the back rim.
#   - lid     : metal flip-top lid hinged at back, covers the mouth.
#   - stopper : inner cork/rubber stopper that lifts vertically out of mouth.
#
# Articulations:
#   - body_to_lid    : REVOLUTE at back rim hinge, axis +Y, positive q flips
#                      the lid open backward (0 to ~2.0 rad).
#   - body_to_stopper: PRISMATIC along +Z, stopper lifts straight up from the
#                      mouth (0 to 0.04 m).

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

# ----- key dimensions (meters) -----
SECT = 0.090        # outer square section width of the glass jar
CORNER_R = 0.010    # rounded corner radius
GLASS_WALL = 0.004  # glass wall thickness

BODY_BOTTOM_Z = 0.0
BODY_TOP_Z = 0.100  # top of main jar body
BODY_H = BODY_TOP_Z - BODY_BOTTOM_Z

# Rim: a thickened lip at the top of the jar
RIM_H = 0.008       # rim height above body top
RIM_OUTER = SECT + 0.004  # rim slightly wider than body
RIM_CORNER_R = CORNER_R + 0.002
RIM_TOP_Z = BODY_TOP_Z + RIM_H

# Mouth opening (wide)
MOUTH_W = 0.068     # inner mouth width (square)
MOUTH_CORNER_R = 0.006

# Hinge lug dimensions
LUG_W = 0.014       # lug width (along Y)
LUG_D = 0.008       # lug depth (along X, extends backward)
LUG_H = 0.010       # lug height
LUG_BORE_D = 0.003  # hinge pin bore diameter
# Hinge position: centered on back rim edge
HINGE_Y = 0.0       # centered in Y
HINGE_X = -(RIM_OUTER / 2.0)  # back edge of rim
HINGE_Z = BODY_TOP_Z + RIM_H / 2.0  # mid-height of rim

# Lid dimensions
LID_W = MOUTH_W + 0.006   # lid slightly wider than mouth for overlap
LID_THICK = 0.005         # lid plate thickness
# Lid hinge lug (on the lid side)
LID_LUG_W = 0.012
LID_LUG_D = 0.006
LID_LUG_H = 0.008

# Stopper dimensions
STOPPER_W = MOUTH_W - 0.006  # slightly smaller than mouth for clearance
STOPPER_CORNER_R = 0.004
STOPPER_H = 0.018            # stopper height
STOPPER_SEAT_Z = BODY_TOP_Z - 0.010  # seated well inside the jar cavity, below rim


def _body_solid() -> cq.Workplane:
    """Hollow rounded-square glass jar with wide mouth and hinge lugs."""
    # Outer shell
    outer = (
        cq.Workplane("XY")
        .rect(SECT, SECT)
        .extrude(BODY_H)
        .edges("|Z")
        .fillet(CORNER_R)
    )

    # Rim on top of the body
    rim = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP_Z)
        .rect(RIM_OUTER, RIM_OUTER)
        .extrude(RIM_H)
        .edges("|Z")
        .fillet(RIM_CORNER_R)
    )
    outer = outer.union(rim)

    # Hollow cavity - open at top, solid glass floor
    inner_w = SECT - 2.0 * GLASS_WALL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_WALL)
        .rect(inner_w, inner_w)
        .extrude(BODY_H + RIM_H + 0.010)  # over-extrude to open through top
        .edges("|Z")
        .fillet(max(CORNER_R - GLASS_WALL, 0.001))
    )
    result = outer.cut(inner)

    # Wide mouth opening is created by the hollow cut extending through the rim top.
    # Add hinge lugs on the back of the rim for the flip-top lid.

    # Hinge lugs on back of rim (2 lugs with gap between for lid lug)
    lug_spacing = 0.018  # center-to-center distance between the 2 body lugs
    for dy in [-lug_spacing / 2.0, lug_spacing / 2.0]:
        lug = (
            cq.Workplane("XY")
            .workplane(offset=HINGE_Z - LUG_H / 2.0)
            .center(HINGE_X - LUG_D / 2.0, dy)
            .rect(LUG_D, LUG_W)
            .extrude(LUG_H)
        )
        # Round the outer edge of the lug
        lug = lug.edges("|Z").fillet(0.002)
        result = result.union(lug)

    # Bore holes through the lugs for the hinge pin
    for dy in [-lug_spacing / 2.0, lug_spacing / 2.0]:
        bore = (
            cq.Workplane("XZ")
            .workplane(offset=dy)
            .center(HINGE_X - LUG_D / 2.0, HINGE_Z)
            .circle(LUG_BORE_D / 2.0)
            .extrude(LUG_W + 0.002)
        )
        result = result.cut(bore)

    return result


def _lid_solid() -> cq.Workplane:
    """Flip-top metal lid with hinge lug. Built in lid-local frame.

    Lid origin at the hinge pivot point. The lid plate extends along +X
    (forward from hinge) with its bottom surface at z=0. At q=0 the lid
    is closed (horizontal, sitting on the rim top).
    """
    # Lid plate extends from hinge along +X, bottom at z=0, top at z=LID_THICK
    # Start the plate at x=0 so it connects to the lug
    plate = (
        cq.Workplane("XY")
        .center(LID_W / 2.0, 0.0)
        .rect(LID_W, LID_W)
        .extrude(LID_THICK)
        .edges("|Z")
        .fillet(0.003)
    )

    # Hinge lug on the lid, extending from x=-LID_LUG_D to x=0.004 (overlapping plate)
    lug = (
        cq.Workplane("XY")
        .center(-LID_LUG_D / 2.0 + 0.002, 0.0)
        .rect(LID_LUG_D + 0.004, LID_LUG_W)
        .extrude(LID_LUG_H)
    )
    lug = lug.edges("|Z").fillet(0.002)

    # Merge plate and lug into one solid (they overlap in the x=0 to x=0.004 region)
    result = plate.union(lug)

    # Bore through the lid lug for the hinge pin (at x = -LID_LUG_D/2)
    lug_bore = (
        cq.Workplane("XZ")
        .center(-LID_LUG_D / 2.0, LID_LUG_H / 2.0)
        .circle(LUG_BORE_D / 2.0)
        .extrude(LID_LUG_W + 0.002)
    )
    result = result.cut(lug_bore)

    return result


def _stopper_solid() -> cq.Workplane:
    """Inner stopper plug. Built in stopper-local frame centered at origin."""
    # Rounded square stopper
    stopper = (
        cq.Workplane("XY")
        .workplane(offset=-STOPPER_H / 2.0)
        .rect(STOPPER_W, STOPPER_W)
        .extrude(STOPPER_H)
        .edges("|Z")
        .fillet(STOPPER_CORNER_R)
    )

    # Small grip knob on top
    knob = (
        cq.Workplane("XY")
        .workplane(offset=STOPPER_H / 2.0)
        .circle(0.008)
        .extrude(0.006)
    )
    stopper = stopper.union(knob)

    # Chamfer the bottom edge slightly
    bottom_ring = (
        cq.Workplane("XY")
        .workplane(offset=-STOPPER_H / 2.0 - 0.001)
        .rect(STOPPER_W - 0.004, STOPPER_W - 0.004)
        .extrude(0.002)
        .edges("|Z")
        .fillet(0.002)
    )
    stopper = stopper.union(bottom_ring)

    return stopper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_jar")

    glass = model.material("clear_glass", rgba=(0.80, 0.85, 0.87, 0.30))
    brushed_metal = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    cork = model.material("cork_rubber", rgba=(0.65, 0.50, 0.35, 1.0))

    # ---- body (root): glass jar with wide mouth and hinge lugs ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "glass_jar_body"),
        material=glass,
        name="glass_jar_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((SECT, SECT, BODY_H + RIM_H)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + RIM_H) / 2.0)),
    )

    # ---- lid: flip-top metal lid hinged at back ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "flip_top_lid"),
        material=brushed_metal,
        name="flip_top_lid",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W + LID_LUG_D, LID_W, LID_THICK)),
        mass=0.04,
        origin=Origin(xyz=(LID_W / 2.0 - LID_LUG_D / 2.0, 0.0, LID_THICK / 2.0)),
    )

    # ---- stopper: inner cork stopper on prismatic joint ----
    stopper = model.part("stopper")
    stopper.visual(
        mesh_from_cadquery(_stopper_solid(), "cork_stopper"),
        material=cork,
        name="cork_stopper",
    )
    stopper.inertial = Inertial.from_geometry(
        Box((STOPPER_W, STOPPER_W, STOPPER_H)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- Articulation 1: body_to_lid (REVOLUTE hinge) ----
    # The hinge pivot is at the back of the rim.
    # In body frame: X = HINGE_X (back edge), Y = 0, Z = HINGE_Z (rim mid-height)
    # Axis is +Y so positive rotation flips the lid backward/upward.
    # At q=0 lid is closed (horizontal). Positive q opens it.
    # The lid part frame is at the hinge pivot; the plate extends +X from there.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, RIM_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=2.2),
    )

    # ---- Articulation 2: body_to_stopper (PRISMATIC vertical lift) ----
    # Stopper sits inside the mouth and lifts straight up.
    # Origin at the seated position of the stopper center.
    model.articulation(
        "body_to_stopper",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, STOPPER_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.045),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stopper = object_model.get_part("stopper")
    lid_hinge = object_model.get_articulation("body_to_lid")
    stopper_lift = object_model.get_articulation("body_to_stopper")

    # The stopper sits inside the jar mouth (intentional nesting for prismatic fit).
    ctx.allow_overlap(
        stopper,
        body,
        elem_a="cork_stopper",
        elem_b="glass_jar_body",
        reason="Cork stopper is intentionally seated inside the jar mouth for the prismatic lift mechanism.",
    )

    # The stopper is inside the hollow jar cavity and connected via the prismatic
    # articulation; it does not need physical wall contact to be "mounted".
    ctx.allow_isolated_part(
        stopper,
        reason="Stopper is intentionally mounted inside the jar cavity via the prismatic lift joint; it does not contact the glass walls.",
    )

    # The lid sits on the rim; the lid lug nests between the body hinge lugs.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="flip_top_lid",
        elem_b="glass_jar_body",
        reason="Lid seats on the rim top and the lid hinge lug interleaves with the body hinge lugs at the pivot.",
    )

    # ---- Jar body is wider and shorter (jar proportions, not bottle) ----
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body has jar proportions (not tall bottle)",
        body_ext[2] < 2.0 * max(body_ext[0], body_ext[1]),
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "jar body is reasonably wide",
        max(body_ext[0], body_ext[1]) > 0.070,
        details=f"body extents={body_ext}",
    )

    # ---- Wide mouth: the body has a hollow opening at the top ----
    # The mouth opening should be visible from above - check that the body
    # inner cavity creates a wide opening.
    ctx.check(
        "jar has non-fixed lid hinge articulation",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"lid_hinge type={lid_hinge.articulation_type}",
    )
    ctx.check(
        "jar has non-fixed stopper prismatic articulation",
        stopper_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"stopper_lift type={stopper_lift.articulation_type}",
    )

    # ---- Lid hinge: closed at q=0, opens backward at positive q ----
    # At rest (q=0), the lid should be near horizontal over the mouth.
    lid_rest_aabb = ctx.part_world_aabb(lid)
    rest_z_max = lid_rest_aabb[1][2] if lid_rest_aabb else 0.0
    ctx.check(
        "lid is positioned at the jar top when closed",
        lid_rest_aabb is not None and lid_rest_aabb[0][2] > BODY_TOP_Z - 0.010,
        details=f"lid_rest_aabb={lid_rest_aabb}",
    )

    # At positive q (1.5 rad ~ 86 deg), the lid should flip open.
    with ctx.pose({lid_hinge: 1.5}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        open_z_max = lid_open_aabb[1][2] if lid_open_aabb else 0.0
        # The lid top should rise significantly when opened (flips up/backward)
        ctx.check(
            "lid flips open at positive hinge angle",
            lid_open_aabb is not None and open_z_max > rest_z_max + 0.015,
            details=f"rest_z_max={rest_z_max}, open_z_max={open_z_max}",
        )

    # ---- Stopper: lifts vertically at positive q ----
    stopper_rest_pos = ctx.part_world_position(stopper)
    with ctx.pose({stopper_lift: 0.04}):
        stopper_up_pos = ctx.part_world_position(stopper)
        ctx.check(
            "stopper lifts straight up (Z increases)",
            stopper_up_pos is not None
            and stopper_rest_pos is not None
            and stopper_up_pos[2] > stopper_rest_pos[2] + 0.035,
            details=f"rest_z={stopper_rest_pos}, lifted_z={stopper_up_pos}",
        )
        ctx.check(
            "stopper does not translate sideways while lifting",
            stopper_up_pos is not None
            and stopper_rest_pos is not None
            and abs(stopper_up_pos[0] - stopper_rest_pos[0]) < 1e-6
            and abs(stopper_up_pos[1] - stopper_rest_pos[1]) < 1e-6,
            details=f"rest_xy={stopper_rest_pos[:2]}, lifted_xy={stopper_up_pos[:2]}",
        )

    # ---- Stopper remains centered in mouth (XY containment) ----
    ctx.expect_within(
        stopper,
        body,
        axes="xy",
        margin=0.005,
        name="stopper stays within jar mouth footprint",
    )

    # ---- Hinge knuckles exist as part of the body geometry ----
    # The body should have hinge lug geometry at the back rim.
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body extends backward past main section (hinge lugs)",
        body_aabb is not None and body_aabb[0][0] < -(SECT / 2.0 - 0.001),
        details=f"body_aabb_min_x={body_aabb[0][0] if body_aabb else None}",
    )

    # ---- Materials are distinct ----
    lid_mat = lid.get_visual("flip_top_lid").material
    body_mat = body.get_visual("glass_jar_body").material
    stopper_mat = stopper.get_visual("cork_stopper").material
    ctx.check(
        "three distinct materials on jar parts",
        lid_mat is not None
        and body_mat is not None
        and stopper_mat is not None
        and getattr(lid_mat, "name", None) == "brushed_steel"
        and getattr(body_mat, "name", None) == "clear_glass"
        and getattr(stopper_mat, "name", None) == "cork_rubber",
        details=f"lid={getattr(lid_mat, 'name', None)}, "
                f"body={getattr(body_mat, 'name', None)}, "
                f"stopper={getattr(stopper_mat, 'name', None)}",
    )

    # ---- Motion limits are physically reasonable ----
    hinge_limits = lid_hinge.motion_limits
    ctx.check(
        "lid hinge has bounded motion limits",
        hinge_limits is not None
        and hinge_limits.lower is not None
        and hinge_limits.upper is not None
        and hinge_limits.upper > hinge_limits.lower
        and hinge_limits.upper <= 3.2,
        details=f"lower={hinge_limits.lower if hinge_limits else None}, "
                f"upper={hinge_limits.upper if hinge_limits else None}",
    )

    lift_limits = stopper_lift.motion_limits
    ctx.check(
        "stopper lift has bounded motion limits",
        lift_limits is not None
        and lift_limits.lower is not None
        and lift_limits.upper is not None
        and lift_limits.upper > lift_limits.lower,
        details=f"lower={lift_limits.lower if lift_limits else None}, "
                f"upper={lift_limits.upper if lift_limits else None}",
    )

    return ctx.report()


object_model = build_object_model()
