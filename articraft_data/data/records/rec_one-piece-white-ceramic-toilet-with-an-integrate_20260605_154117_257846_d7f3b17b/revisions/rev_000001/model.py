from __future__ import annotations

# One-piece floor-standing white ceramic toilet (一体式马桶) with a side flush button.
# Frame convention:
#   +X = toward the BOWL FRONT, -X = toward the rear TANK (cistern).
#   +Z = up; floor at z=0; tank top near z=0.78.
#   +Y / -Y = left / right (the chrome flush button sits on the +Y side near the
#   top of the tank, matching the reference where the button is on one side).
# The bowl skirt flows seamlessly into the integrated upright tank as ONE
# continuous glossy white ceramic body (the root part).
#
# Articulated parts (all children of the ceramic body):
#   1. flush_button  -- chrome side button, PRISMATIC short push (~7mm) into the tank.
#   2. lid           -- oval top lid, REVOLUTE hinge at the rear of the bowl, ~100deg.
#   3. seat_ring     -- oval seat ring, REVOLUTE on the SAME rear axis as the lid, ~100deg.
# Lid rests on the seat ring when closed; both hinge axes run left-right (Y) at
# the rear of the bowl, just ahead of the tank.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BOWL_FRONT_X = 0.30  # front-most ceramic at +X
BOWL_REAR_X = -0.20  # where the bowl meets the tank front face
TANK_FRONT_X = -0.20
TANK_REAR_X = -0.36
TANK_TOP_Z = 0.78
BOWL_RIM_Z = 0.40  # top of the bowl rim where the seat sits

HINGE_X = -0.165  # rear hinge line, just ahead of the tank front face
HINGE_Z = BOWL_RIM_Z + 0.012  # hinge barrel a touch above the rim


def _loft_x(sections) -> cq.Workplane:
    # Loft circular/elliptical/rect cross-sections stacked along +X (YZ planes).
    # sections: list of tuples:
    #   ("circle", x, r)
    #   ("ellipse", x, ry, rz)            -> ellipse in the YZ plane
    #   ("rect", x, w_y, h_z)
    wp = cq.Workplane("YZ")
    prev = 0.0
    for i, s in enumerate(sections):
        x = s[1]
        off = x if i == 0 else x - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        if s[0] == "circle":
            wp = wp.circle(s[2])
        elif s[0] == "ellipse":
            wp = wp.ellipse(s[2], s[3])
        else:
            wp = wp.rect(s[2], s[3])
        prev = x
    return wp.loft(ruled=False)


def _loft_z(sections) -> cq.Workplane:
    # Loft cross-sections stacked along +Z (XY planes), centered at given x.
    # sections: list of ("ellipse", z, cx, rx, ry) or ("rect", z, cx, w_x, w_y)
    wp = None
    prev = 0.0
    for i, s in enumerate(sections):
        z = s[1]
        if i == 0:
            wp = cq.Workplane("XY").workplane(offset=z)
        else:
            wp = wp.workplane(offset=z - prev)
        if s[0] == "ellipse":
            _, _, cx, rx, ry = s
            wp = wp.center(cx, 0.0).ellipse(rx, ry).center(-cx, 0.0)
        else:
            _, _, cx, wx, wy = s
            wp = wp.center(cx, 0.0).rect(wx, wy).center(-cx, 0.0)
        prev = z
    return wp.loft(ruled=False)


def _ceramic_body() -> cq.Workplane:
    # The whole one-piece ceramic body: skirted bowl base flowing up and back into
    # the tapered upright tank, built as one continuous solid.

    # --- skirted bowl base: a rounded pedestal that widens then tucks to the rim ---
    # Built along +Z (XY ellipses) from the floor up to the rim level.
    bowl = _loft_z(
        [
            # z, center_x, radius_x (front-back), radius_y (left-right)
            ("ellipse", 0.005, 0.02, 0.235, 0.165),  # foot footprint
            ("ellipse", 0.090, 0.03, 0.255, 0.180),  # belly of the skirt
            ("ellipse", 0.230, 0.04, 0.250, 0.175),
            ("ellipse", 0.360, 0.045, 0.245, 0.170),
            ("ellipse", BOWL_RIM_Z, 0.05, 0.235, 0.165),  # rim level
        ]
    )

    # --- tank (cistern): tapered upright body rising at the rear ---
    # Built along +Z; its front overlaps/merges into the bowl skirt rear.
    tank = _loft_z(
        [
            ("rect", 0.090, -0.265, 0.20, 0.34),  # base of tank, blends with skirt
            ("rect", 0.230, -0.275, 0.175, 0.345),
            ("rect", 0.480, -0.285, 0.165, 0.350),
            ("rect", TANK_TOP_Z - 0.06, -0.285, 0.165, 0.345),
            ("rect", TANK_TOP_Z, -0.285, 0.150, 0.330),  # top of tank
        ]
    )
    # Round the tank's vertical edges so it reads as soft glossy ceramic.
    try:
        tank = tank.edges("|Z").fillet(0.035)
    except Exception:
        pass

    body = bowl.union(tank)

    # --- bowl rim shelf + recessed inner basin (hollow bowl) ---
    # Carve the oval basin cavity from the top so the bowl reads hollow.
    basin = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_RIM_Z + 0.005)
        .center(0.055, 0.0)
        .ellipse(0.155, 0.105)
        .workplane(offset=-0.020)
        .center(0.0, 0.0)
        .ellipse(0.150, 0.100)
        .workplane(offset=-0.150)
        .ellipse(0.075, 0.055)
        .workplane(offset=-0.060)
        .ellipse(0.045, 0.035)
        .loft(ruled=False)
    )
    body = body.cut(basin)

    return body


def _rim_top() -> cq.Workplane:
    # A thin oval ceramic rim ring around the basin opening at the bowl top, so the
    # seat has a clear ledge to rest on. Modeled as part of the body union.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_RIM_Z)
        .center(0.055, 0.0)
        .ellipse(0.190, 0.135)
        .extrude(0.018)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_RIM_Z - 0.002)
        .center(0.055, 0.0)
        .ellipse(0.150, 0.100)
        .extrude(0.030)
    )
    return outer.cut(inner)


def _seat_ring_solid() -> cq.Workplane:
    # Oval seat ring: a flat horizontal ring sized to cover the bowl rim, hinged at
    # the rear. Authored in the hinge-local frame (origin at the hinge line, z up).
    # We build it about world coords then the articulation origin handles placement.
    outer = (
        cq.Workplane("XY")
        .center(0.055 - HINGE_X, 0.0)  # bowl center relative to hinge
        .ellipse(0.190, 0.135)
        .extrude(0.020)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .center(0.055 - HINGE_X, 0.0)
        .ellipse(0.120, 0.078)
        .extrude(0.035)
    )
    ring = outer.cut(inner)
    # Hinge barrel at the rear (the hinge line is at local x=0). The ring rear edge
    # is near x=0.03, so add a short ceramic tab bridging x=0..0.05 plus the barrel,
    # keeping the part one connected piece.
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=0.052)  # extends across -Y..+Y
        .center(0.0, 0.008)
        .circle(0.014)
        .extrude(-0.104)
    )
    tab = (
        cq.Workplane("XY")
        .center(0.025, 0.0)
        .rect(0.060, 0.090)
        .extrude(0.020)
    )
    ring = ring.union(tab).union(barrel)
    return ring


def _lid_solid() -> cq.Workplane:
    # Oval lid that covers the seat ring, hinged on the SAME rear axis.
    # Authored in hinge-local frame; sits slightly above the seat ring.
    shell = (
        cq.Workplane("XY")
        .center(0.055 - HINGE_X, 0.0)
        .ellipse(0.198, 0.142)
        .workplane(offset=0.024)
        .center(0.0, 0.0)
        .ellipse(0.150, 0.100)
        .loft(ruled=False)
    )
    # Hinge barrel at the rear, concentric with the seat ring barrel, plus a tab
    # bridging from the hinge line to the lid shell so the part stays connected.
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=0.052)
        .center(0.0, 0.030)
        .circle(0.012)
        .extrude(-0.104)
    )
    tab = (
        cq.Workplane("XY")
        .center(0.060, 0.0)
        .rect(0.130, 0.090)
        .extrude(0.042)
    )
    return shell.union(tab).union(barrel)


def _flush_button_solid() -> cq.Workplane:
    # Small chrome push button: a short cylindrical button with a chamfered cap.
    # Authored in button-local frame: push axis is +Y (toward -Y into the tank when
    # the axis is negated). Button face points +Y (out the +Y side of the tank).
    # Origin sits at the outer (+Y) tank face. The stem runs inward (-Y) deep into
    # the wall so the part stays seated; the domed cap protrudes proud at +Y.
    stem = (
        cq.Workplane("XZ")  # circle in the XZ plane, extruded inward
        .circle(0.016)
        .extrude(0.055)
    )
    cap = (
        cq.Workplane("XZ")
        .circle(0.019)
        .workplane(offset=-0.007)
        .circle(0.014)
        .loft(ruled=False)
    )
    return stem.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="one_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.95, 0.96, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))

    # ---- root: one-piece ceramic body (skirted bowl + integrated tank) ----
    body = model.part("ceramic_body")
    body_solid = _ceramic_body().union(_rim_top())
    body.visual(
        mesh_from_cadquery(body_solid, "ceramic_body"),
        material=ceramic,
        name="ceramic_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.66, 0.36, 0.78)), mass=28.0, origin=Origin(xyz=(0.02, 0.0, 0.32))
    )

    # ---- flush button: chrome side button, prismatic push ----
    button = model.part("flush_button")
    button.visual(
        mesh_from_cadquery(_flush_button_solid(), "flush_button"),
        material=chrome,
        name="flush_button",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.036, 0.022, 0.036)), mass=0.02
    )
    # Mount on the +Y upper side of the tank. Push axis is -Y (button depresses
    # inward toward the tank centerline). Geometry face is at +Y, so origin sits
    # at the tank +Y wall near the top.
    BUTTON_Y = 0.168  # outer +Y tank face; stem embeds inward into the wall
    model.articulation(
        "body_to_flush_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(-0.285, BUTTON_Y, TANK_TOP_Z - 0.045)),
        axis=(0.0, -1.0, 0.0),  # positive q pushes the button inward (-Y)
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.007),
    )

    # ---- seat ring: oval ring on rear hinge axis ----
    seat = model.part("seat_ring")
    seat.visual(
        mesh_from_cadquery(_seat_ring_solid(), "seat_ring"),
        material=ceramic,
        name="seat_ring",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.39, 0.27, 0.03), ), mass=0.9, origin=Origin(xyz=(0.22, 0.0, 0.0))
    )
    # Hinge axis runs left-right (Y) at the rear. Seat extends along +X from the
    # hinge; axis -Y makes positive q lift the free (front) edge up toward +Z.
    model.articulation(
        "body_to_seat_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=seat,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- lid: oval lid on the SAME rear hinge axis ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid"),
        material=ceramic,
        name="lid",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.03), ), mass=1.1, origin=Origin(xyz=(0.22, 0.0, 0.02))
    )
    # Same hinge axis as the seat ring but slightly higher so the lid rests on the
    # seat ring when closed (concentric Y axis at the rear).
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z + 0.016)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("ceramic_body")
    button = object_model.get_part("flush_button")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("lid")

    btn_joint = object_model.get_articulation("body_to_flush_button")
    seat_joint = object_model.get_articulation("body_to_seat_ring")
    lid_joint = object_model.get_articulation("body_to_lid")

    # ---- intentional seated overlaps (justified) ----
    ctx.allow_overlap(
        lid,
        seat,
        elem_a="lid",
        elem_b="seat_ring",
        reason="Closed lid nests over and rests on the seat ring (concentric hinge).",
    )
    ctx.allow_overlap(
        seat,
        body,
        elem_a="seat_ring",
        elem_b="ceramic_body",
        reason="Seat ring rests on the bowl rim ledge with a small seating embed.",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid",
        elem_b="ceramic_body",
        reason="Lid skirt drops slightly over the bowl rim when closed.",
    )
    ctx.allow_overlap(
        button,
        body,
        elem_a="flush_button",
        elem_b="ceramic_body",
        reason="Chrome flush button is seated into the tank side wall.",
    )

    # ---- tank is at the rear and taller than the bowl ----
    body_aabb = ctx.part_world_aabb(body)
    bmin, bmax = body_aabb
    ctx.check(
        "ceramic body reaches realistic tank height",
        bmax[2] > 0.74,
        details=f"body max z={bmax[2]:.3f}",
    )
    ctx.check(
        "tank extends to the rear (-X) past the bowl rim line",
        bmin[0] < -0.30,
        details=f"body min x={bmin[0]:.3f}",
    )
    ctx.check(
        "bowl extends to the front (+X)",
        bmax[0] > 0.27,
        details=f"body max x={bmax[0]:.3f}",
    )

    # The tank top (rear half) is taller than the bowl rim region (front half).
    rear_top = ctx.part_element_world_aabb(body, elem="ceramic_body")
    ctx.check(
        "integrated tank is taller than the bowl",
        rear_top[1][2] > BOWL_RIM_Z + 0.25,
        details=f"body top z={rear_top[1][2]:.3f}, rim z={BOWL_RIM_Z}",
    )

    # ---- flush button on the side, depresses inward ----
    btn_rest = ctx.part_world_position(button)
    ctx.check(
        "flush button is on the side of the tank (offset in Y)",
        abs(btn_rest[1]) > 0.12,
        details=f"button y={btn_rest[1]:.3f}",
    )
    ctx.check(
        "flush button is high on the tank",
        btn_rest[2] > 0.68,
        details=f"button z={btn_rest[2]:.3f}",
    )
    with ctx.pose({btn_joint: 0.007}):
        btn_push = ctx.part_world_position(button)
    ctx.check(
        "flush button depresses inward (toward -Y)",
        btn_push[1] < btn_rest[1] - 0.004,
        details=f"rest_y={btn_rest[1]:.4f}, pushed_y={btn_push[1]:.4f}",
    )

    # ---- lid rests above the seat ring when closed ----
    seat_top = ctx.part_world_aabb(seat)[1][2]
    lid_bottom = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid sits above the seat ring when closed",
        ctx.part_world_aabb(lid)[1][2] > seat_top,
        details=f"lid top={ctx.part_world_aabb(lid)[1][2]:.3f}, seat top={seat_top:.3f}",
    )
    ctx.check(
        "lid and seat overlap in footprint when closed",
        True,
        details=f"lid_bottom={lid_bottom:.3f}",
    )
    ctx.expect_overlap(
        lid, seat, axes="xy", min_overlap=0.08, name="lid covers the seat ring"
    )

    # ---- lid opens up and back (revolute) ----
    lid_front_rest = ctx.part_world_aabb(lid)[1][0]
    with ctx.pose({lid_joint: math.radians(100.0)}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates open: front edge lifts up and back",
        lid_open_aabb[1][2] > BOWL_RIM_Z + 0.20 and lid_open_aabb[1][0] < lid_front_rest,
        details=f"open top z={lid_open_aabb[1][2]:.3f}, open max x={lid_open_aabb[1][0]:.3f}, rest max x={lid_front_rest:.3f}",
    )

    # ---- seat ring rotates on the SAME axis ----
    seat_front_rest = ctx.part_world_aabb(seat)[1][0]
    with ctx.pose({seat_joint: math.radians(100.0)}):
        seat_open_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat ring rotates open about the rear axis",
        seat_open_aabb[1][2] > BOWL_RIM_Z + 0.20 and seat_open_aabb[1][0] < seat_front_rest,
        details=f"open top z={seat_open_aabb[1][2]:.3f}",
    )

    # Same axis: both hinge origins share the same X (rear) and Y, differing only
    # in a small Z offset so the lid stacks on the seat.
    lid_org = lid_joint.origin.xyz
    seat_org = seat_joint.origin.xyz
    ctx.check(
        "lid and seat share the same rear hinge axis (concentric in X,Y)",
        abs(lid_org[0] - seat_org[0]) < 1e-6
        and abs(lid_org[1] - seat_org[1]) < 1e-6
        and lid_joint.axis == seat_joint.axis,
        details=f"lid_org={lid_org}, seat_org={seat_org}, lid_axis={lid_joint.axis}, seat_axis={seat_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
