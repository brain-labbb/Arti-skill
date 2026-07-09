from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global layout (meters), world frame: +X = front, +Z = up.
# Body outer envelope: 0.50 deep (X) x 0.50 wide (Y) x 0.55 tall (Z).
# The door is hinged on the +Y side of the opening (viewer's right when
# facing the front, matching the reference image) and swings outward (+X).
# ---------------------------------------------------------------------------

WALL = 0.045
BODY_X = 0.50
BODY_Y = 0.50
BODY_Z = 0.55

HINGE_X = 0.262  # hinge axis sits slightly proud of the front face (x=0.25)
HINGE_Y = 0.2275  # centered on the right wall thickness
HINGE_Z = BODY_Z / 2.0

DOOR_THICK = 0.065
DOOR_W = 0.395  # door leaf width (local Y)
DOOR_H = 0.45
DOOR_CY = -0.225  # door leaf center in hinge-local Y
DOOR_X_OUT = -0.007  # outer face of door slab in hinge-local X
DOOR_X_IN = DOOR_X_OUT - DOOR_THICK
DOOR_EDGE_Y = DOOR_CY - DOOR_W / 2.0  # free (latch-side) edge, local Y

BOLT_ZS = (-0.14, 0.0, 0.14)  # in hinge/mid-height local Z
BOLT_R = 0.016
BOLT_LEN = 0.075
BOLT_STROKE = 0.032
POCKET_R = 0.018

KNUCKLE_R = 0.014
KNUCKLE_L = 0.06
DOOR_KNUCKLE_ZS = (-0.12, 0.0, 0.12)
BODY_KNUCKLE_ZS = (0.215, 0.335)  # world Z

DIAL_Z = 0.10  # hinge-local Z of dial center on door face
HANDLE_Z = -0.08
FACE_CY = DOOR_CY  # dial/handle centered on the door leaf


def _ingot_mesh():
    """Trapezoid-section gold ingot, base 0.055 x 0.14, 0.034 tall, base at z=0."""
    solid = (
        cq.Workplane("XY")
        .rect(0.055, 0.14)
        .workplane(offset=0.034)
        .rect(0.042, 0.118)
        .loft(combine=True)
    )
    return mesh_from_cadquery(solid, "gold_ingot")


def _latch_wall_mesh():
    """Latch-side wall with three bored strike pockets that receive the door bolts."""
    wall = cq.Workplane("XY").box(BODY_X, WALL, BODY_Z)
    for zr in BOLT_ZS:
        pocket = cq.Solid.makeCylinder(
            POCKET_R,
            0.06,
            cq.Vector(0.2225, -0.0175, zr),
            cq.Vector(0.0, 1.0, 0.0),
        )
        wall = wall.cut(pocket)
    return mesh_from_cadquery(wall, "side_wall_latch")


FOOT_R = 0.022
FOOT_H = 0.03


def _leveling_foot_mesh():
    """Short brushed-steel leveling foot: cylinder with a chamfered base edge."""
    foot = (
        cq.Workplane("XY")
        .circle(FOOT_R)
        .extrude(FOOT_H)
    )
    # Chamfer the bottom edge so the foot reads as a machined pad.
    foot = foot.edges("<Z").chamfer(0.003)
    return mesh_from_cadquery(foot, "leveling_foot")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_security_safe")

    charcoal = model.material("matte_charcoal_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    door_gray = model.material("door_gray_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    interior_gray = model.material("interior_gray", rgba=(0.46, 0.47, 0.48, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    polished = model.material("polished_steel", rgba=(0.80, 0.82, 0.85, 1.0))
    silver = model.material("silver_ring", rgba=(0.85, 0.86, 0.88, 1.0))
    black = model.material("dial_black", rgba=(0.05, 0.05, 0.06, 1.0))
    gold = model.material("gold", rgba=(0.85, 0.66, 0.14, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("safe_body")
    body.visual(
        Box((WALL, BODY_Y, BODY_Z)),
        origin=Origin(xyz=(-0.2275, 0.0, HINGE_Z)),
        material=charcoal,
        name="back_wall",
    )
    body.visual(
        Box((BODY_X, WALL, BODY_Z)),
        origin=Origin(xyz=(0.0, 0.2275, HINGE_Z)),
        material=charcoal,
        name="side_wall_hinge",
    )
    body.visual(
        _latch_wall_mesh(),
        origin=Origin(xyz=(0.0, -0.2275, HINGE_Z)),
        material=charcoal,
        name="side_wall_latch",
    )
    body.visual(
        Box((BODY_X, BODY_Y, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z - WALL / 2.0)),
        material=charcoal,
        name="top_wall",
    )
    body.visual(
        Box((BODY_X, BODY_Y, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=charcoal,
        name="bottom_wall",
    )
    # Interior shelf splits the cavity into two compartments. It embeds 1 mm
    # into each side wall so it reads as a welded-in shelf.
    body.visual(
        Box((0.365, 0.412, 0.022)),
        origin=Origin(xyz=(-0.0225, 0.0, HINGE_Z)),
        material=interior_gray,
        name="shelf",
    )

    # Body-side hinge knuckles, carried by straps welded to the front face of
    # the hinge-side wall. The knuckle barrels stack flush with the door-side
    # knuckles along the hinge axis.
    for i, zk in enumerate(BODY_KNUCKLE_ZS):
        body.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_L),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, zk)),
            material=brushed,
            name=f"body_knuckle_{i}",
        )
        body.visual(
            Box((0.025, 0.025, 0.05)),
            origin=Origin(xyz=(HINGE_X - 0.0075, HINGE_Y, zk)),
            material=charcoal,
            name=f"body_knuckle_strap_{i}",
        )

    # Static gold-bar props: a 2x2 + 1 stack on the safe floor and a 3 + 2
    # stack on the shelf. Each bar embeds ~1 mm into its support.
    ingot = _ingot_mesh()
    floor_positions = [
        (-0.14, -0.08, 0.044),
        (-0.14, 0.08, 0.044),
        (-0.07, -0.08, 0.044),
        (-0.07, 0.08, 0.044),
        (-0.105, 0.0, 0.077),
    ]
    shelf_positions = [
        (-0.11, -0.125, 0.285),
        (-0.11, 0.0, 0.285),
        (-0.11, 0.125, 0.285),
        (-0.11, -0.0625, 0.318),
        (-0.11, 0.0625, 0.318),
    ]
    for i, xyz in enumerate(floor_positions + shelf_positions):
        body.visual(ingot, origin=Origin(xyz=xyz), material=gold, name=f"gold_ingot_{i}")

    # Leveling feet: four short brushed-steel cylinders under the bottom wall,
    # one near each corner of the body footprint. Each foot is flush-mounted
    # under the bottom wall so the body sits raised on the feet.
    foot_mesh = _leveling_foot_mesh()
    foot_inset = BODY_X / 2.0 - WALL / 2.0 - FOOT_R  # inset from outer edges
    foot_corners = [
        (-foot_inset, -foot_inset),
        (-foot_inset, foot_inset),
        (foot_inset, -foot_inset),
        (foot_inset, foot_inset),
    ]
    for i, (fx, fy) in enumerate(foot_corners):
        body.visual(
            foot_mesh,
            origin=Origin(xyz=(fx, fy, -FOOT_H)),
            material=brushed,
            name=f"leveling_foot_{i}",
        )

    # ------------------------------------------------------------------ door
    # Door part frame sits on the hinge axis; the leaf extends along local -Y.
    door = model.part("door")
    door.visual(
        Box((DOOR_THICK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_X_OUT - DOOR_THICK / 2.0, DOOR_CY, 0.0)),
        material=door_gray,
        name="door_slab",
    )
    # Raised inner liner panel on the cavity side of the door.
    door.visual(
        Box((0.012, 0.34, 0.39)),
        origin=Origin(xyz=(DOOR_X_IN - 0.005, DOOR_CY, 0.0)),
        material=door_gray,
        name="door_inner_panel",
    )
    # Door-side hinge knuckles plus straps that wrap in front of the body
    # face and tie into the door slab.
    for i, zk in enumerate(DOOR_KNUCKLE_ZS):
        door.visual(
            Cylinder(radius=KNUCKLE_R, length=KNUCKLE_L),
            origin=Origin(xyz=(0.0, 0.0, zk)),
            material=brushed,
            name=f"hinge_knuckle_{i}",
        )
        door.visual(
            Box((0.024, 0.040, 0.05)),
            origin=Origin(xyz=(0.002, -0.015, zk)),
            material=door_gray,
            name=f"hinge_strap_{i}",
        )
    # Raised circular bosses for the dial and the handle wheel, embedded
    # 2 mm into the door slab so they read as machined-on hubs.
    door.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(DOOR_X_OUT + 0.006, FACE_CY, DIAL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=door_gray,
        name="dial_boss",
    )
    door.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(DOOR_X_OUT + 0.006, FACE_CY, HANDLE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=door_gray,
        name="handle_boss",
    )
    # Fixed index mark above the dial for reading the combination.
    door.visual(
        Box((0.003, 0.005, 0.009)),
        origin=Origin(xyz=(DOOR_X_OUT + 0.001, FACE_CY, DIAL_Z + 0.055)),
        material=black,
        name="dial_index_mark",
    )

    # --------------------------------------------------------- bolt carriage
    # Brushed-steel locking bolt carriage: three bolts tied together by a
    # vertical bar hidden inside the door slab. The carriage slides along the
    # door width; extended bolts seat into bored pockets in the latch wall.
    carriage = model.part("bolt_carriage")
    carriage.visual(
        Box((0.018, 0.05, 0.32)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=brushed,
        name="carriage_bar",
    )
    for i, zr in enumerate(BOLT_ZS):
        carriage.visual(
            Cylinder(radius=BOLT_R, length=BOLT_LEN),
            origin=Origin(xyz=(0.0, -0.001, zr), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"lock_bolt_{i}",
        )

    # ------------------------------------------------------------------ dial
    # Combination dial: silver numbered ring with tick marks, black fluted
    # dial knob, spinning continuously about the door-normal axis.
    dial = model.part("combination_dial")
    dial.visual(
        Cylinder(radius=0.047, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=silver,
        name="dial_ring",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.022,
            body_style="cylindrical",
            edge_radius=0.0012,
            grip=KnobGrip(style="fluted", count=24, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "dial_knob",
    )
    dial.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=black,
        name="dial_knob",
    )
    for i in range(12):
        theta = i * pi / 6.0
        dial.visual(
            Box((0.009, 0.0022, 0.0024)),
            origin=Origin(
                xyz=(0.040 * cos(theta), 0.040 * sin(theta), 0.007),
                rpy=(0.0, 0.0, theta),
            ),
            material=black,
            name=f"dial_tick_{i}",
        )

    # ---------------------------------------------------------------- handle
    # Polished four-spoke handle wheel with ball ends on a central hub.
    handle = model.part("handle_wheel")
    handle.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=polished,
        name="handle_hub",
    )
    handle.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=polished,
        name="hub_cap",
    )
    for i in range(4):
        theta = pi / 4.0 + i * pi / 2.0
        handle.visual(
            Cylinder(radius=0.0085, length=0.125),
            origin=Origin(
                xyz=(0.0625 * cos(theta), 0.0625 * sin(theta), 0.020),
                rpy=(0.0, pi / 2.0, theta),
            ),
            material=polished,
            name=f"spoke_{i}",
        )
        handle.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.125 * cos(theta), 0.125 * sin(theta), 0.020)),
            material=polished,
            name=f"spoke_knob_{i}",
        )

    # ---------------------------------------------------------- articulations
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        # Positive q swings the free edge outward (+X); ~120 degrees max.
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=2.094),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(DOOR_X_OUT + 0.014, FACE_CY, HANDLE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-pi / 2.0, upper=pi / 2.0),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DOOR_X_OUT + 0.014, FACE_CY, DIAL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    # Bolt carriage rides inside the door; in the real lockwork the handle
    # retracts it (the SDK forbids angular->linear mimics, so it is driven
    # directly). Positive q throws the bolts toward the latch edge; the rest
    # pose q=0 is mid-stroke with the bolts seated in the strike pockets.
    model.articulation(
        "bolt_slide",
        ArticulationType.PRISMATIC,
        parent=door,
        child=carriage,
        origin=Origin(xyz=(DOOR_X_OUT - DOOR_THICK / 2.0, -0.406, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.2,
            lower=-BOLT_STROKE / 2.0,
            upper=BOLT_STROKE / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("combination_dial")
    handle = object_model.get_part("handle_wheel")
    carriage = object_model.get_part("bolt_carriage")
    hinge = object_model.get_articulation("door_hinge")
    handle_spin = object_model.get_articulation("handle_spin")
    dial_spin = object_model.get_articulation("dial_spin")
    bolt_slide = object_model.get_articulation("bolt_slide")

    # The carriage is intentionally a nested fit sliding inside the solid
    # door-slab proxy; the bolts pass through the slab to the latch edge.
    ctx.allow_overlap(
        carriage,
        door,
        elem_a="carriage_bar",
        elem_b="door_slab",
        reason="Bolt carriage bar intentionally slides inside the solid door slab proxy.",
    )
    for i in range(3):
        ctx.allow_overlap(
            carriage,
            door,
            elem_a=f"lock_bolt_{i}",
            elem_b="door_slab",
            reason="Locking bolt intentionally passes through the solid door slab proxy.",
        )

    # Carriage stays captured inside the door slab footprint on the
    # non-motion axes.
    ctx.expect_within(
        carriage,
        door,
        axes="xz",
        inner_elem="carriage_bar",
        outer_elem="door_slab",
        margin=0.001,
        name="bolt carriage bar stays captured inside the door slab",
    )

    # Hero mounts: dial and handle are seated on their door bosses.
    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_ring",
        elem_b="dial_boss",
        contact_tol=5e-4,
        name="dial ring seats on its raised boss",
    )
    ctx.expect_overlap(
        dial,
        door,
        axes="yz",
        elem_a="dial_ring",
        elem_b="dial_boss",
        min_overlap=0.08,
        name="dial ring is centered on its boss",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_hub",
        elem_b="handle_boss",
        contact_tol=5e-4,
        name="handle hub seats on its raised boss",
    )

    # Hinge knuckle stack: door knuckles and body knuckles meet along the
    # shared hinge axis.
    ctx.expect_contact(
        door,
        body,
        elem_a="hinge_knuckle_0",
        elem_b="body_knuckle_0",
        contact_tol=5e-4,
        name="door knuckle stacks against body knuckle",
    )

    # Closed door sits inside the opening with a real latch-side gap that the
    # bolts bridge into the strike pockets.
    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_slab",
        negative_elem="side_wall_latch",
        min_gap=0.004,
        max_gap=0.02,
        name="closed door leaves a latch-side jamb gap",
    )
    ctx.expect_within(
        door,
        body,
        axes="yz",
        margin=0.001,
        name="closed door stays within the body envelope",
    )

    # Envelope: cabinet box ~0.5 x 0.5 x 0.55 m (hinge knuckles sit slightly
    # proud of the front face, so measure the depth on a wall element).
    aabb = ctx.part_world_aabb(body)
    wall_aabb = ctx.part_element_world_aabb(body, elem="side_wall_hinge")
    top_aabb = ctx.part_element_world_aabb(body, elem="top_wall")
    bottom_aabb = ctx.part_element_world_aabb(body, elem="bottom_wall")
    ctx.check(
        "safe body envelope is about 0.5 x 0.5 x 0.55 m",
        aabb is not None
        and wall_aabb is not None
        and top_aabb is not None
        and bottom_aabb is not None
        and abs((wall_aabb[1][0] - wall_aabb[0][0]) - 0.50) < 0.01
        and abs((aabb[1][1] - aabb[0][1]) - 0.50) < 0.01
        and abs((top_aabb[1][2] - bottom_aabb[0][2]) - 0.55) < 0.01,
        details=f"body aabb={aabb}, hinge wall aabb={wall_aabb}, top={top_aabb}, bottom={bottom_aabb}",
    )

    # Interior shelf splits the cavity at mid height and spans the full width.
    shelf_aabb = ctx.part_element_world_aabb(body, elem="shelf")
    ctx.check(
        "interior shelf sits at mid height and spans the cavity",
        shelf_aabb is not None
        and 0.26 < (shelf_aabb[0][2] + shelf_aabb[1][2]) / 2.0 < 0.29
        and (shelf_aabb[1][1] - shelf_aabb[0][1]) > 0.40,
        details=f"shelf aabb={shelf_aabb}",
    )

    # Gold bar props rest on the safe floor and on the shelf, not floating.
    ingot_floor = ctx.part_element_world_aabb(body, elem="gold_ingot_0")
    ingot_shelf = ctx.part_element_world_aabb(body, elem="gold_ingot_5")
    ctx.check(
        "gold bars are seated on the floor and shelf",
        ingot_floor is not None
        and 0.040 < ingot_floor[0][2] < 0.047
        and ingot_shelf is not None
        and 0.281 < ingot_shelf[0][2] < 0.288,
        details=f"floor ingot={ingot_floor}, shelf ingot={ingot_shelf}",
    )

    # At rest (mid-stroke) the bolts protrude past the door edge and are
    # engaged through the jamb gap into the latch-wall strike pockets.
    bolt_rest = ctx.part_element_world_aabb(carriage, elem="lock_bolt_1")
    ctx.check(
        "rest pose: bolts protrude and engage the strike pockets",
        bolt_rest is not None and -0.24 < bolt_rest[0][1] < -0.206,
        details=f"mid bolt aabb={bolt_rest}",
    )

    # Sliding the carriage to its lower limit retracts the bolts clear of the
    # jamb so the door can swing.
    with ctx.pose({bolt_slide: -0.016}):
        bolt_open = ctx.part_element_world_aabb(carriage, elem="lock_bolt_1")
        ctx.check(
            "retracted carriage pulls the bolts clear of the jamb",
            bolt_open is not None
            and bolt_open[0][1] > -0.205
            and bolt_rest is not None
            and bolt_open[0][1] > bolt_rest[0][1] + 0.012,
            details=f"mid bolt retracted aabb={bolt_open}",
        )

    # The handle wheel rotates about the door-normal axis: a spoke ball end
    # visibly orbits the hub.
    ball_rest = ctx.part_element_world_aabb(handle, elem="spoke_knob_0")
    with ctx.pose({handle_spin: pi / 2.0}):
        ball_turned = ctx.part_element_world_aabb(handle, elem="spoke_knob_0")
        ctx.check(
            "handle spoke orbits when the handle turns",
            ball_rest is not None
            and ball_turned is not None
            and abs(
                (ball_turned[0][2] + ball_turned[1][2]) / 2.0
                - (ball_rest[0][2] + ball_rest[1][2]) / 2.0
            )
            > 0.05,
            details=f"ball rest={ball_rest}, turned={ball_turned}",
        )

    # Door opens outward about the vertical hinge on the +Y (right) side:
    # at ~70 deg the leaf center moves forward (+X) and toward the hinge side.
    slab_rest = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({hinge: 1.22}):
        slab_open = ctx.part_element_world_aabb(door, elem="door_slab")
        ctx.check(
            "door swings outward about the right-side vertical hinge",
            slab_rest is not None
            and slab_open is not None
            and (slab_open[0][0] + slab_open[1][0]) / 2.0
            > (slab_rest[0][0] + slab_rest[1][0]) / 2.0 + 0.15
            and (slab_open[0][1] + slab_open[1][1]) / 2.0
            > (slab_rest[0][1] + slab_rest[1][1]) / 2.0 + 0.05,
            details=f"slab rest={slab_rest}, open={slab_open}",
        )

    # Dial spins about the door-normal axis: a rim tick visibly orbits.
    tick_rest = ctx.part_element_world_aabb(dial, elem="dial_tick_0")
    with ctx.pose({dial_spin: pi / 2.0}):
        tick_spun = ctx.part_element_world_aabb(dial, elem="dial_tick_0")
        ctx.check(
            "dial tick orbits when the dial spins",
            tick_rest is not None
            and tick_spun is not None
            and abs(
                (tick_spun[0][2] + tick_spun[1][2]) / 2.0
                - (tick_rest[0][2] + tick_rest[1][2]) / 2.0
            )
            > 0.025,
            details=f"tick rest={tick_rest}, spun={tick_spun}",
        )

    # Leveling feet: four feet under the body, one at each corner, raising the
    # body above the floor.
    foot_aabbs = [
        ctx.part_element_world_aabb(body, elem=f"leveling_foot_{i}") for i in range(4)
    ]
    ctx.check(
        "four leveling feet exist under the body",
        all(aabb is not None for aabb in foot_aabbs),
        details=f"foot aabbs={foot_aabbs}",
    )

    # Each foot sits flush under the bottom wall: foot top at z≈0, and feet
    # extend below to z≈-FOOT_H, raising the body off the floor.
    if all(aabb is not None for aabb in foot_aabbs):
        foot_tops = [aabb[1][2] for aabb in foot_aabbs]
        foot_bottoms = [aabb[0][2] for aabb in foot_aabbs]
        ctx.check(
            "feet are flush under the bottom wall and raise the body",
            all(-0.002 < ft < 0.002 for ft in foot_tops)
            and all(fb < -0.025 for fb in foot_bottoms)
            and bottom_aabb is not None
            and abs(bottom_aabb[0][2]) < 0.002,
            details=f"foot tops={foot_tops}, foot bottoms={foot_bottoms}, bottom wall bottom={bottom_aabb[0][2] if bottom_aabb else None}",
        )

        # Feet are near the four corners of the body footprint.
        foot_centers_xy = [
            ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)
            for aabb in foot_aabbs
        ]
        ctx.check(
            "feet are placed near the four corners of the body footprint",
            all(abs(cx) > 0.15 and abs(cy) > 0.15 for cx, cy in foot_centers_xy),
            details=f"foot centers xy={foot_centers_xy}",
        )

    return ctx.report()


object_model = build_object_model()
