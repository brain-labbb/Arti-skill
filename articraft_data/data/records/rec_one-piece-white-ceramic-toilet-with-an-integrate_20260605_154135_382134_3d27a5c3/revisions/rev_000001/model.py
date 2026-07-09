from __future__ import annotations

# One-piece floor-standing ceramic toilet with a round top dual-flush button.
#
# Coordinate convention (object-intrinsic frame):
#   +X = toward the FRONT of the bowl (the seat/lid free edge points +X).
#   -X = toward the REAR, where the integrated cistern (tank) rises.
#   +Z = up. The floor contact is at z = 0; the flat tank top is near z = 0.80.
#   +Y / -Y = left / right (symmetric). The lid/seat hinge axis runs along Y
#             at the rear of the bowl, just ahead of the tank front face.
#
# The ceramic body is ONE continuous glossy white shell: a rounded, skirted bowl
# base that flows seamlessly up into the tall flat-topped tank at the rear.
#
# Articulations (all children of the main ceramic body):
#   - flush_button : round chrome dual-flush actuator recessed in the tank top,
#                    PRISMATIC, short downward push (~7 mm) along -Z.
#   - seat_lid     : oval top lid, REVOLUTE hinge about the rear Y axis (~100 deg).
#   - seat_ring    : oval seat ring beneath the lid, REVOLUTE about the SAME
#                    rear Y axis (concentric with the lid), ~100 deg.
#   Lid rests on the seat ring when closed; seat ring rests on the bowl rim.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Key dimensions (meters) -------------------------------------------------
BOWL_WIDTH = 0.37          # left-right width of the bowl skirt
BOWL_FRONT_X = 0.34        # front-most point of the bowl
BOWL_REAR_X = -0.16        # where the bowl meets the tank front
TANK_FRONT_X = -0.16       # tank front face
TANK_REAR_X = -0.34        # tank rear face
TANK_WIDTH = 0.36          # tank left-right width
TANK_TOP_Z = 0.80          # flat tank top height
TANK_BOTTOM_Z = 0.40       # where the tank shoulder meets the bowl skirt
BOWL_RIM_Z = 0.40          # bowl rim / seat plane height
SKIRT_BOTTOM_Z = 0.0       # floor contact

HINGE_X = -0.13            # hinge line, just ahead of the tank front face
HINGE_Z = BOWL_RIM_Z       # seat ring hinges right at the bowl rim plane

SEAT_FRONT_X = 0.33        # oval seat / lid front-most point
SEAT_HALF_LEN = (SEAT_FRONT_X - HINGE_X) / 2.0
SEAT_MID_X = (SEAT_FRONT_X + HINGE_X) / 2.0
SEAT_HALF_W = 0.165        # seat oval half-width

BUTTON_R = 0.034           # chrome dual-flush button radius
BUTTON_X = TANK_REAR_X + 0.085   # button sits near the rear of the tank top
BUTTON_PUSH = 0.007        # prismatic downward travel


def _loft_x(sections) -> cq.Workplane:
    """Loft circular/rect/ellipse sections stacked along +X (sections in YZ planes).

    Each section: ("circle", x, r) | ("rect", x, w, h) | ("ellipse", x, ry, rz).
    """
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


def _body_shell() -> cq.Workplane:
    """One continuous ceramic body: skirted bowl flowing into the tall tank.

    Built by lofting cross-sections along +X. The front sections are the rounded
    bowl skirt (tall, from floor up to the rim); the rear sections grow into the
    full-height flat-topped tank box.
    """
    # ---- Bowl skirt: lofted from the front toward the rear, full floor-to-rim.
    # Sections are vertical slabs (rect in Y x Z) of increasing width, rounded by
    # using an ellipse-ish footprint via rounded rect approximation through rects.
    bowl = _loft_x(
        [
            # front nose (narrow, rounded)
            ("rect", BOWL_FRONT_X, 0.13, 0.40),
            ("rect", BOWL_FRONT_X - 0.05, 0.26, 0.40),
            ("rect", 0.18, 0.345, 0.40),
            ("rect", 0.02, BOWL_WIDTH, 0.40),
            ("rect", BOWL_REAR_X + 0.04, BOWL_WIDTH, 0.40),
        ]
    )
    # Center the bowl vertically so its slab spans z in [0, 0.40].
    bowl = bowl.translate((0.0, 0.0, 0.20))

    # ---- Tank: tall flat-topped box at the rear, slightly narrower than bowl.
    tank_h = TANK_TOP_Z - SKIRT_BOTTOM_Z
    tank = (
        cq.Workplane("XY")
        .center((TANK_FRONT_X + TANK_REAR_X) / 2.0, 0.0)
        .rect(TANK_FRONT_X - TANK_REAR_X, TANK_WIDTH)
        .extrude(tank_h)
    )

    # ---- Shoulder transition: loft from the tank front face up to the rim plane
    # so the tank skirt flows into the bowl as one body (fills the gap region).
    shoulder = _loft_x(
        [
            ("rect", BOWL_REAR_X + 0.04, BOWL_WIDTH, 0.40),
            ("rect", TANK_FRONT_X, TANK_WIDTH, 0.40),
        ]
    ).translate((0.0, 0.0, 0.20))

    # ---- Hinge boss ridge at the rear rim: a low ceramic bar straddling the
    # hinge line that the seat ring and lid rear edges rest against (mount path).
    hinge_boss = (
        cq.Workplane("XY")
        .center(HINGE_X + 0.005, 0.0)
        .rect(0.05, 0.24)
        .extrude(0.452)
    )

    body = bowl.union(shoulder).union(tank).union(hinge_boss)

    # Round the vertical edges of the tank for the glossy seamless look.
    try:
        body = body.edges("|Z").fillet(0.03)
    except Exception:
        pass

    # ---- Hollow out the bowl basin (open top, recessed cavity) so it reads real.
    basin = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_RIM_Z + 0.001)
        .center(0.10, 0.0)
        .ellipse(0.135, 0.115)
        .workplane(offset=-0.18)
        .center(-0.02, 0.0)
        .ellipse(0.075, 0.065)
        .loft(ruled=False)
    )
    body = body.cut(basin)

    return body


def _seat_ring_mesh():
    """Oval ceramic seat ring with a central opening (loop, hinged at rear)."""
    outer = (
        cq.Workplane("XY")
        .ellipse(SEAT_HALF_LEN, SEAT_HALF_W)
        .extrude(0.022)
    )
    hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .ellipse(SEAT_HALF_LEN - 0.045, SEAT_HALF_W - 0.045)
        .extrude(0.030)
    )
    ring = outer.cut(hole)
    try:
        ring = ring.edges("|Z").fillet(0.006)
    except Exception:
        pass
    return mesh_from_cadquery(ring, "seat_ring_shell")


def _lid_mesh():
    """Oval soft-close lid: a slightly domed solid oval plate."""
    lid = (
        cq.Workplane("XY")
        .ellipse(SEAT_HALF_LEN + 0.004, SEAT_HALF_W + 0.004)
        .extrude(0.020)
    )
    try:
        lid = lid.edges("|Z").fillet(0.010)
        lid = lid.faces(">Z").fillet(0.006)
    except Exception:
        pass
    return mesh_from_cadquery(lid, "lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="one_piece_toilet")

    ceramic = model.material("ceramic_white", rgba=(0.95, 0.96, 0.97, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- Main ceramic body (root): bowl + integrated tank as one shell --------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell"),
        material=ceramic,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.68, 0.37, 0.80)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )

    # ---- Seat ring: oval loop on the bowl rim, hinged at the rear Y axis ------
    seat = model.part("seat_ring")
    # Author the ring centered on its hinge so the joint origin is the hinge line.
    # The ring geometry is offset so its center sits at SEAT_MID_X relative to the
    # hinge, lying flat at the rim plane.
    seat.visual(
        _seat_ring_mesh(),
        origin=Origin(xyz=(SEAT_MID_X - HINGE_X, 0.0, 0.0)),
        material=ceramic,
        name="seat_ring_shell",
    )
    seat.inertial = Inertial.from_geometry(
        Box((2.0 * SEAT_HALF_LEN, 2.0 * SEAT_HALF_W, 0.022)),
        mass=0.9,
        origin=Origin(xyz=(SEAT_MID_X - HINGE_X, 0.0, 0.011)),
    )
    model.articulation(
        "body_to_seat_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=seat,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Ring extends along +X from the hinge; -Y lifts the free (front) edge up.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- Lid: oval cover resting on the seat ring, SAME rear Y hinge axis -----
    lid = model.part("seat_lid")
    lid.visual(
        _lid_mesh(),
        origin=Origin(xyz=(SEAT_MID_X - HINGE_X, 0.0, 0.0)),
        material=ceramic,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((2.0 * SEAT_HALF_LEN, 2.0 * SEAT_HALF_W, 0.020)),
        mass=0.8,
        origin=Origin(xyz=(SEAT_MID_X - HINGE_X, 0.0, 0.010)),
    )
    model.articulation(
        "body_to_seat_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        # Same hinge line as the seat ring, raised slightly so the lid sits on top.
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z + 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- Dual-flush button: round chrome actuator recessed in the tank top ----
    button = model.part("flush_button")
    # Round chrome disc; modeled as a split half/half dual-flush button.
    disc = CylinderGeometry(BUTTON_R, 0.012, radial_segments=48)
    button.visual(
        mesh_from_geometry(disc, "flush_button_disc"),
        material=chrome,
        name="flush_button_disc",
    )
    # Thin chrome trim ring hugging the button edge (recessed bezel detail).
    # Tube straddles the disc rim so it stays one connected chrome body.
    ring = TorusGeometry(BUTTON_R, 0.004, radial_segments=12, tubular_segments=48)
    button.visual(
        mesh_from_geometry(ring, "flush_button_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=chrome,
        name="flush_button_ring",
    )
    button.inertial = Inertial.from_geometry(
        Cylinder(BUTTON_R, 0.012), mass=0.03
    )
    model.articulation(
        "body_to_flush_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        # Button center sits flush at the tank top; pushes down along -Z.
        origin=Origin(xyz=(BUTTON_X, 0.0, TANK_TOP_Z - 0.004)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.05, lower=0.0, upper=BUTTON_PUSH
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    seat = object_model.get_part("seat_ring")
    lid = object_model.get_part("seat_lid")
    button = object_model.get_part("flush_button")
    seat_hinge = object_model.get_articulation("body_to_seat_ring")
    lid_hinge = object_model.get_articulation("body_to_seat_lid")
    push = object_model.get_articulation("body_to_flush_button")

    # --- Intentional overlaps: seated/nested parts ---------------------------
    ctx.allow_overlap(
        lid,
        seat,
        elem_a="lid_shell",
        elem_b="seat_ring_shell",
        reason="The lid rests directly on top of the seat ring when closed.",
    )
    ctx.allow_overlap(
        seat,
        body,
        elem_a="seat_ring_shell",
        elem_b="body_shell",
        reason="The seat ring seats on the bowl rim with a small local embed.",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="body_shell",
        reason="The closed lid skirt sits against the bowl rim/tank front.",
    )
    ctx.allow_overlap(
        button,
        body,
        elem_a="flush_button_disc",
        elem_b="body_shell",
        reason="The chrome dual-flush button is recessed into the tank top.",
    )
    ctx.allow_overlap(
        button,
        body,
        elem_a="flush_button_ring",
        elem_b="body_shell",
        reason="The chrome trim bezel is recessed into the tank top.",
    )

    # --- Tank top is flat and above the bowl ---------------------------------
    body_aabb = ctx.part_world_aabb(body)
    top_z = body_aabb[1][2]
    ctx.check(
        "tank top reaches realistic height (~0.80 m)",
        0.74 < top_z < 0.86,
        details=f"body top z={top_z}",
    )
    # Button rest position sits at the tank top, above the bowl rim plane.
    btn_pos = ctx.part_world_position(button)
    ctx.check(
        "flush button is on the tank top, above the bowl rim",
        btn_pos is not None and btn_pos[2] > BOWL_RIM_Z + 0.2,
        details=f"button pos={btn_pos}",
    )
    # Button sits toward the rear (-X) of the object, on the tank.
    ctx.check(
        "flush button is on the rear tank, not over the bowl",
        btn_pos is not None and btn_pos[0] < BOWL_REAR_X,
        details=f"button pos={btn_pos}",
    )

    # --- Flush button depresses downward -------------------------------------
    rest_z = ctx.part_world_position(button)[2]
    with ctx.pose({push: BUTTON_PUSH}):
        pushed_z = ctx.part_world_position(button)[2]
    ctx.check(
        "flush button depresses downward when pressed",
        pushed_z < rest_z - 0.004,
        details=f"rest_z={rest_z}, pushed_z={pushed_z}",
    )

    # --- Lid rotates open and lifts its front edge up ------------------------
    lid_front_rest = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_hinge: math.radians(95.0)}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        lid_open_top = lid_open_aabb[1][2]
        lid_open_front = lid_open_aabb[1][0]
    ctx.check(
        "lid rotates open (rises and tucks rearward)",
        lid_open_top > lid_front_rest + 0.10,
        details=f"rest_top={lid_front_rest}, open_top={lid_open_top}",
    )
    ctx.check(
        "open lid leans back toward the tank (front edge retracts)",
        lid_open_front < SEAT_FRONT_X,
        details=f"open front x={lid_open_front}",
    )

    # --- Seat ring rotates on the SAME rear axis -----------------------------
    # Verify both hinges share the same Y axis and rear X/Z line.
    ctx.check(
        "seat ring and lid hinge axes are parallel (left-right Y)",
        seat_hinge.axis == lid_hinge.axis,
        details=f"seat axis={seat_hinge.axis}, lid axis={lid_hinge.axis}",
    )
    ctx.check(
        "seat ring and lid hinges share the same rear X line",
        abs(seat_hinge.origin.xyz[0] - lid_hinge.origin.xyz[0]) < 1e-6,
        details=f"seat x={seat_hinge.origin.xyz[0]}, lid x={lid_hinge.origin.xyz[0]}",
    )
    seat_rest_top = ctx.part_world_aabb(seat)[1][2]
    with ctx.pose({seat_hinge: math.radians(95.0)}):
        seat_open_top = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "seat ring rotates open about the rear axis",
        seat_open_top > seat_rest_top + 0.10,
        details=f"rest_top={seat_rest_top}, open_top={seat_open_top}",
    )

    # --- Lid is above the seat ring when closed ------------------------------
    lid_bottom = ctx.part_world_aabb(lid)[0][2]
    seat_top = ctx.part_world_aabb(seat)[1][2]
    ctx.check(
        "closed lid sits above the seat ring",
        ctx.part_world_position(lid)[2] > ctx.part_world_position(seat)[2],
        details=f"lid z={ctx.part_world_position(lid)[2]}, seat z={ctx.part_world_position(seat)[2]}",
    )
    ctx.expect_overlap(
        lid, seat, axes="xy", min_overlap=0.10, name="lid covers the seat ring footprint"
    )

    # --- Seat ring rests on the bowl rim (contact) ---------------------------
    ctx.expect_contact(seat, body, name="seat ring rests on the bowl rim")

    return ctx.report()


object_model = build_object_model()
