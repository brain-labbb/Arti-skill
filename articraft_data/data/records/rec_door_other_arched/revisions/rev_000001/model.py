from __future__ import annotations

# Rustic arched wooden plank door with black wrought-iron strap hinges and
# studded iron hardware, set inside a rough carved stone archway.
#
# Conventions used here:
# - World up is +Z. The stone archway is the fixed root and rests on the ground
#   plane at z = 0, extending upward.
# - The door leaf is an UPRIGHT panel. It swings on a VERTICAL (Z) hinge axis at
#   the left jamb, so the closed door stands vertical and opens horizontally.
# - The door leaf is authored in a local frame whose origin lies on the hinge
#   line (local x = 0 at the hinge edge); the leaf body extends along +X toward
#   the latch edge and +Z upward. Iron straps/studs/boss are fused onto the
#   leaf because they rigidly move with it.
# - The iron ring pull is its OWN articulated part: it hangs from the mounting
#   boss and lifts away from the door on a horizontal (door-local X) revolute
#   axis through the boss.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------

# Door leaf. The arched top is a true semicircle of radius LEAF_W/2 springing
# at LEAF_SPRING, so the apex is LEAF_SPRING + LEAF_W/2.
LEAF_W = 0.90          # width (along +X from hinge edge to latch edge)
LEAF_SPRING = 1.55     # height where the semicircular arch starts springing
LEAF_TOP = LEAF_SPRING + LEAF_W / 2.0  # apex height (= 2.00 m)
LEAF_THK = 0.055       # plank thickness (along Y)
PLANK_COUNT = 6        # number of vertical plank boards

# Opening / clearance: the stone opening is the door profile expanded outward by
# GAP everywhere. Because the arch is a semicircle, the opening shares the door's
# arch CENTER (same springline) and uses radius = LEAF_W/2 + GAP. This keeps the
# two arches concentric so the faceted meshes never touch. A few-cm rough reveal
# gap is realistic for a hand-carved stone archway.
GAP = 0.035            # gap between leaf edge and stone reveal
OPEN_W = LEAF_W + 2.0 * GAP        # clear opening width
OPEN_SPRING = LEAF_SPRING          # same springline -> concentric arch center
OPEN_TOP = OPEN_SPRING + OPEN_W / 2.0  # opening apex (semicircle)

# Stone frame
JAMB_W = 0.26          # stone jamb width (depth into wall direction, +/-X of opening)
WALL_THK = 0.34        # frame thickness through the wall (along Y)
FRAME_TOP = 2.45       # top of the stone arch keystone block

# The opening's left reveal sits at world X = X_REVEAL_L; the door hinge edge is
# at that reveal plus the clearance gap.
X_REVEAL_L = 0.0
X_OPEN_L = X_REVEAL_L + JAMB_W          # inner face of left jamb (left reveal)
X_OPEN_R = X_OPEN_L + OPEN_W            # inner face of right jamb (right reveal)
X_REVEAL_R = X_OPEN_R + JAMB_W          # outer right edge of frame

# Door hinge line is at the left reveal + clearance.
X_HINGE = X_OPEN_L + GAP

# Ring-pull handle (door-local frame). The ring hangs from a mounting boss; the
# pivot is at the TOP of the ring centerline where it threads through the boss.
RING_CX = LEAF_W - 0.16          # ring center, X (near the latch edge)
RING_CZ = 1.10                   # ring center, Z
RING_MEAN_R = 0.055              # ring center-line radius
RING_TUBE_R = 0.009              # ring bar (tube) radius
RING_PIVOT_Y = LEAF_THK / 2.0 + 0.012  # ring plane sits proud of the wood face
RING_PIVOT_Z = RING_CZ + RING_MEAN_R   # top of the ring = hinge point on the boss


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, authored in meters)
# ---------------------------------------------------------------------------


def _arched_profile_face(width: float, spring: float, top: float) -> cq.Workplane:
    """A 2D face in the XZ plane: rectangle up to `spring`, then a semicircular
    arch from `spring` to `top` (apex). Returned on the XZ workplane spanning
    x in [0, width], z in [0, top]."""
    radius = width / 2.0
    cx = width / 2.0
    # Outline points: start bottom-left, up the left side to the spring line,
    # arc over the top, down the right side, back along the bottom.
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, spring)
        .threePointArc((cx, top), (width, spring))
        .lineTo(width, 0.0)
        .close()
    )
    return wp


def build_door_leaf_mesh():
    """Build the rustic arched plank door leaf as a single fused CadQuery solid.

    Local frame: x in [0, LEAF_W] (0 = hinge edge), y in [-LEAF_THK/2, +LEAF_THK/2],
    z in [0, LEAF_TOP]. Includes vertical plank seams, two cross battens (ledges),
    and a slightly recessed back so the planked face reads on the front (+Y).
    """
    # Main arched plank slab. `both=True` extrudes symmetrically about the XZ
    # plane so the slab straddles y=0 with total thickness LEAF_THK.
    slab = _arched_profile_face(LEAF_W, LEAF_SPRING, LEAF_TOP).extrude(
        LEAF_THK / 2.0, both=True
    )

    leaf = slab

    # Vertical plank seams: shallow V-grooves on the front face (+Y) between boards.
    groove_depth = 0.008
    groove_w = 0.010
    board_pitch = LEAF_W / PLANK_COUNT
    for i in range(1, PLANK_COUNT):
        x = i * board_pitch
        groove = (
            cq.Workplane("XZ")
            .moveTo(x, 0.0)
            .rect(groove_w, LEAF_TOP * 2.2, centered=True)
            .extrude(groove_depth)
        )
        # Groove block spans y in [-groove_depth, 0]; shift it up to +Y so its
        # outer face sits flush with the front surface (+LEAF_THK/2) and it cuts
        # a shallow seam into the front of the leaf.
        groove = groove.translate((0.0, LEAF_THK / 2.0, 0.0))
        leaf = leaf.cut(groove)

    # Two horizontal cross battens (ledges) on the back (-Y) tying the planks.
    batten_h = 0.12
    batten_thk = 0.025
    for z_center in (0.45, 1.30):
        batten = (
            cq.Workplane("XY")
            .box(LEAF_W * 0.92, batten_thk, batten_h)
            .translate(
                (
                    LEAF_W / 2.0,
                    -LEAF_THK / 2.0 - batten_thk / 2.0 + 0.004,  # slight embed for a clean weld
                    z_center,
                )
            )
        )
        leaf = leaf.union(batten)

    return leaf


def build_strap_hinge_mesh(z_center: float):
    """A black wrought-iron strap hinge: a long tapered strap running across the
    door front with a rolled barrel knuckle at the hinge edge (x=0). Authored in
    the door leaf local frame and fused as iron later via a separate visual."""
    strap_thk = 0.007
    strap_w = 0.072          # bolder strap band
    strap_len = 0.80         # runs nearly the full leaf width (LEAF_W = 0.90)
    embed = 0.004  # strap sits proud but bites EMBED into the wood front face
    y_shift = LEAF_THK / 2.0 - embed + strap_thk
    # Tapered strap lying on the front face. It keeps a near-constant bold width
    # most of the way across, then necks down and flares into a decorative
    # spade/fleur terminal so it reads as forged wrought iron, not a plain bar.
    neck = strap_len - 0.10  # where the strap necks before the spade flare
    strap = (
        cq.Workplane("XZ")
        .moveTo(0.10, z_center - strap_w / 2.0)
        .lineTo(0.10, z_center + strap_w / 2.0)
        .lineTo(neck, z_center + strap_w / 2.0)        # full-width run
        .lineTo(neck + 0.025, z_center + 0.040)        # flare out into the spade
        .lineTo(strap_len, z_center + 0.010)           # neck of the spade
        .lineTo(strap_len + 0.055, z_center)           # spade tip
        .lineTo(strap_len, z_center - 0.010)
        .lineTo(neck + 0.025, z_center - 0.040)
        .lineTo(neck, z_center - strap_w / 2.0)
        .close()
        .extrude(strap_thk)
    )
    strap = strap.translate((0.0, y_shift, 0.0))

    # Inner plate near the barrel, extending onto the door leaf (+x only) so the
    # strap does not lap over the stone jamb when the door is closed.
    plate = (
        cq.Workplane("XZ")
        .moveTo(0.06, z_center)
        .rect(0.12, strap_w, centered=True)
        .extrude(strap_thk)
        .translate((0.0, y_shift, 0.0))
    )
    strap = strap.union(plate)

    # Rolled barrel knuckle wrapping the hinge edge (cylinder along Z at x=0).
    # It is pushed proud of the wood front face (its bore center at
    # y = LEAF_THK/2 + 0.018) so the captured pintle pin never bites the wood,
    # while its underside still overlaps the strap plate so it welds on.
    barrel_y = LEAF_THK / 2.0 + 0.018
    barrel = (
        cq.Workplane("XY")
        .circle(0.018)
        .circle(0.011)
        .extrude(0.11)
        .translate((0.0, barrel_y, z_center - 0.055))
    )
    strap = strap.union(barrel)
    return strap


def build_iron_hardware_mesh():
    """Studded iron hardware on the front: a grid of dome studs plus the ring
    pull's mounting boss near the latch edge. Fused as one iron visual on the
    door leaf. The ring itself is a separate articulated part."""
    studs = None
    stud_r = 0.013
    front_y = LEAF_THK / 2.0

    def _dome(sx: float, sz: float):
        return (
            cq.Workplane("XZ")
            .moveTo(sx, sz)
            .sphere(stud_r)
            .translate((0.0, front_y, 0.0))
        )

    # Dense rows of dome studs marching along the two black strap-hinge bands
    # (clavos pattern) at the strap heights z=0.45 and z=1.30, plus a row up high
    # near the arch and a vertical border row down the latch edge.
    strap_stud_xs = [0.12, 0.24, 0.36, 0.48, 0.60, 0.72]
    for sz in (0.45, 1.30):
        for sx in strap_stud_xs:
            dome = _dome(sx, sz)
            studs = dome if studs is None else studs.union(dome)

    # Upper border row near the springline of the arch.
    for sx in (0.18, 0.36, 0.54, 0.72):
        studs = studs.union(_dome(sx, 1.78))

    # Vertical border column of studs running down the latch (right) edge.
    for sz in (0.30, 0.65, 1.00, 1.45):
        studs = studs.union(_dome(LEAF_W - 0.10, sz))

    # Mounting backplate boss for the ring pull: short disc embedded into the
    # wood, standing proud, placed at the top of the ring so the ring hangs
    # from it. The ring threads through this boss and pivots on it.
    boss = (
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(0.020)  # cylinder along +Z
        .rotate((0, 0, 0), (1, 0, 0), -90)  # lay it along +Y
        .translate((RING_CX, front_y - 0.006, RING_PIVOT_Z))
    )
    studs = studs.union(boss)
    return studs


def build_ring_pull_mesh():
    """The iron ring pull, authored in its own pivot-local frame: the origin
    (0, 0, 0) is the point where the ring threads through the mounting boss,
    and the ring hangs straight down (-Z) lying in the XZ plane (loop normal
    +Y). Lifting rotates it about the local X axis, swinging it out toward +Y
    (away from the door front). Built on XY then revolved about the Y axis."""
    ring = (
        cq.Workplane("XY")
        .center(RING_MEAN_R, 0.0)
        .circle(RING_TUBE_R)
        .revolve(360.0, (-RING_MEAN_R, 0.0, 0.0), (-RING_MEAN_R, 1.0, 0.0))
        .translate((0.0, 0.0, -RING_MEAN_R))
    )
    return ring


def build_stone_frame_mesh():
    """Rough carved stone archway: two jambs and an arched voussoir/keystone top,
    forming an arched opening. Sits on the ground at z=0. Authored in world frame.

    A rectangular stone slab with an arched opening cut out of it. The opening
    matches the door leaf arch with a small clearance.
    """
    block_w = X_REVEAL_R - X_REVEAL_L
    # Solid stone block from ground up to the keystone top.
    block = (
        cq.Workplane("XY")
        .box(block_w, WALL_THK, FRAME_TOP)
        .translate((X_REVEAL_L + block_w / 2.0, 0.0, FRAME_TOP / 2.0))
    )

    # Cut the arched opening through the full wall thickness, centered in Y.
    opening = _arched_profile_face(OPEN_W, OPEN_SPRING, OPEN_TOP).extrude(
        (WALL_THK + 0.10) / 2.0, both=True
    )
    opening = opening.translate((X_OPEN_L, 0.0, 0.0))
    frame = block.cut(opening)

    # Protruding voussoir keystone block at the crown, seated so its underside
    # sits on the opening apex (z = OPEN_TOP) and never dips into the opening
    # where the door arch lives.
    key_h = 0.34
    keystone = (
        cq.Workplane("XY")
        .box(0.22, WALL_THK + 0.06, key_h)
        .translate(((X_OPEN_L + X_OPEN_R) / 2.0, 0.0, OPEN_TOP + key_h / 2.0))
    )
    frame = frame.union(keystone)

    # Rough base plinth blocks at the foot of each jamb (rests on the ground).
    for cx in (X_REVEAL_L + JAMB_W / 2.0, X_OPEN_R + JAMB_W / 2.0):
        plinth = (
            cq.Workplane("XY")
            .box(JAMB_W + 0.06, WALL_THK + 0.05, 0.20)
            .translate((cx, 0.0, 0.10))
        )
        frame = frame.union(plinth)

    return frame


def build_jamb_pintle_mesh(z_center: float):
    """A fixed wrought-iron pintle anchored in the left jamb stone: a bracket arm
    lagged into the jamb reaches out to a vertical pin at the door hinge line.
    The door's rolled strap barrel wraps this pin, so the door hangs on it.
    Authored in world coordinates (part of the fixed stone_frame)."""
    # Pin/barrel bore center plane (proud of the wood front face so the pin only
    # meets the strap barrel, never the wood leaf edge).
    pin_y = LEAF_THK / 2.0 + 0.018
    # Bracket arm: a short bar lagged into the jamb stone (inner end at x<X_OPEN_L)
    # reaching out and forward to support the pin. It bridges from the stone front
    # plane up to the proud pin so the pintle is grounded in the frame.
    arm = (
        cq.Workplane("XY")
        .box(0.18, 0.024, 0.044)
        .translate((X_OPEN_L - 0.03, pin_y, z_center))
    )
    # Vertical pin (the pintle) at the door hinge line; the barrel rotates on it.
    # Pin radius (0.012) slightly exceeds the barrel bore (0.011) so the barrel is
    # captured on the pin with a small intended overlap (allowed in tests).
    pin = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.13)
        .translate((X_HINGE, pin_y, z_center - 0.065))
    )
    # Small ball finial on top of the pin.
    finial = (
        cq.Workplane("XY")
        .sphere(0.012)
        .translate((X_HINGE, pin_y, z_center + 0.065))
    )
    return arm.union(pin).union(finial)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arched_plank_door")

    stone = model.material("stone", rgba=(0.60, 0.59, 0.55, 1.0))
    # Warm mid-tone honey/oak plank wood (lighter than chocolate to match the
    # sunlit timber in the reference photo).
    wood = model.material("oak_plank", rgba=(0.60, 0.43, 0.24, 1.0))
    iron = model.material("wrought_iron", rgba=(0.08, 0.08, 0.09, 1.0))

    # --- Fixed stone archway (root) ---
    frame = model.part("stone_frame")
    frame.visual(
        mesh_from_cadquery(build_stone_frame_mesh(), "stone_frame"),
        material=stone,
        name="stone_arch",
    )
    # Fixed wrought-iron pintles anchored in the left jamb; the door barrels hang
    # on these pins. Heights match the two strap-hinge barrels (z=0.45, 1.30).
    pintles = build_jamb_pintle_mesh(0.45).union(build_jamb_pintle_mesh(1.30))
    frame.visual(
        mesh_from_cadquery(pintles, "jamb_pintles"),
        material=iron,
        name="jamb_pintles",
    )

    # --- Door leaf (planks + battens) ---
    door = model.part("door")
    # The leaf is authored in its own local frame (hinge edge at local x=0).
    door.visual(
        mesh_from_cadquery(build_door_leaf_mesh(), "door_planks"),
        material=wood,
        name="door_planks",
    )
    # Iron strap hinges (two), fused onto the leaf.
    strap_lo = build_strap_hinge_mesh(0.45)
    strap_hi = build_strap_hinge_mesh(1.30)
    door.visual(
        mesh_from_cadquery(strap_lo.union(strap_hi), "strap_hinges"),
        material=iron,
        name="strap_hinges",
    )
    # Studs + ring-pull mounting boss, fused onto the leaf.
    door.visual(
        mesh_from_cadquery(build_iron_hardware_mesh(), "iron_studs"),
        material=iron,
        name="iron_studs",
    )

    # --- Ring pull: its own part, hinged on the mounting boss ---
    ring_pull = model.part("ring_pull")
    ring_pull.visual(
        mesh_from_cadquery(build_ring_pull_mesh(), "ring_pull"),
        material=iron,
        name="ring_pull",
    )

    # --- Hinge: vertical (Z) axis at the left jamb reveal ---
    # The leaf local frame has its hinge edge at local x=0, so the joint origin
    # is placed at the world hinge line and the leaf coincides at q=0. Increasing
    # q swings the free (latch) edge toward -Y (door opens outward to the front),
    # so use axis +Z (right-hand rule: +Z rotation moves +X toward +Y) -> negate
    # to open the +X edge toward -Y / outward. We pick the sign so the door
    # swings clear of the opening.
    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(X_HINGE, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=2.0, lower=0.0, upper=math.radians(110.0)
        ),
    )

    # --- Ring-pull pivot: horizontal (door-local X) axis through the boss ---
    # The ring is authored with its pivot at the local origin, so the joint
    # origin places that pivot at the top of the ring on the boss. At q=0 the
    # ring hangs flat against the door; rotation about +X swings the hanging
    # ring toward +Y (right-hand rule takes -Z toward +Y), lifting it away
    # from the door front.
    model.articulation(
        "door_to_ring_pull",
        ArticulationType.REVOLUTE,
        parent=door,
        child=ring_pull,
        origin=Origin(xyz=(RING_CX, RING_PIVOT_Y, RING_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=0.0, upper=math.radians(90.0)
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("stone_frame")
    door = object_model.get_part("door")
    ring_pull = object_model.get_part("ring_pull")
    hinge = object_model.get_articulation("frame_to_door")
    ring_joint = object_model.get_articulation("door_to_ring_pull")

    # The door hangs on the fixed jamb pintles: the rolled strap barrels (on the
    # door) wrap the jamb pin (on the frame) with a small intended capture
    # overlap. This is the real hinge interface, so allow that element overlap.
    ctx.allow_overlap(
        door,
        frame,
        elem_a="strap_hinges",
        elem_b="jamb_pintles",
        reason=(
            "The door's rolled strap-hinge barrels are captured on the fixed "
            "jamb pintle pins; this is the hinge bearing, an intended local "
            "metal-on-metal embedding."
        ),
    )

    # The ring threads through the mounting boss on the door: this is the ring
    # pull's pivot bearing, an intended local metal-on-metal embedding.
    ctx.allow_overlap(
        ring_pull,
        door,
        elem_a="ring_pull",
        elem_b="iron_studs",
        reason=(
            "The ring pull threads through its mounting boss (part of the "
            "iron_studs visual); this is the pivot bearing the ring rotates on."
        ),
    )

    # Hero features present.
    ctx.check(
        "door has planked leaf + iron straps + studs",
        {v.name for v in door.visuals} >= {"door_planks", "strap_hinges", "iron_studs"},
        details=f"visuals={[v.name for v in door.visuals]}",
    )
    ctx.check(
        "frame has the carved stone arch + jamb pintles",
        {v.name for v in frame.visuals} >= {"stone_arch", "jamb_pintles"},
        details=f"visuals={[v.name for v in frame.visuals]}",
    )

    # The door barrel actually contacts the fixed pintle (real hinge support,
    # not a floating leaf).
    ctx.expect_contact(
        door,
        frame,
        elem_a="strap_hinges",
        elem_b="jamb_pintles",
        name="door barrels hang on the jamb pintles",
    )

    # Hinge is a vertical (Z) revolute, so the upright door swings horizontally
    # rather than lying flat.
    ax = hinge.axis
    ctx.check(
        "hinge axis is vertical (Z)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
        details=f"axis={ax}",
    )

    # Ring pull stays engaged with its mounting boss (real pivot bearing).
    ctx.expect_contact(
        ring_pull,
        door,
        elem_a="ring_pull",
        elem_b="iron_studs",
        name="ring pull threads through its mounting boss",
    )

    # Ring-pull pivot is a horizontal (door-local X) revolute on the boss.
    rax = ring_joint.axis
    ctx.check(
        "ring-pull axis is horizontal (X)",
        abs(rax[0]) > 0.99 and abs(rax[1]) < 1e-6 and abs(rax[2]) < 1e-6,
        details=f"axis={rax}",
    )

    # Lifting the ring swings it out in front of the door (+Y) and raises its
    # lowest point, proving it pivots about the boss rather than translating.
    with ctx.pose({ring_joint: 0.0}):
        rb_rest = ctx.part_world_aabb(ring_pull)
    with ctx.pose({ring_joint: math.radians(80.0)}):
        rb_lift = ctx.part_world_aabb(ring_pull)
    ctx.check(
        "lifted ring swings out away from the door front",
        rb_lift[1][1] > rb_rest[1][1] + 0.05,
        details=f"rest_max_y={rb_rest[1][1]:.3f} lifted_max_y={rb_lift[1][1]:.3f}",
    )
    ctx.check(
        "lifted ring's lowest point rises toward the pivot height",
        rb_lift[0][2] > rb_rest[0][2] + 0.05,
        details=f"rest_min_z={rb_rest[0][2]:.3f} lifted_min_z={rb_lift[0][2]:.3f}",
    )

    # Frame rests on the ground (z >= ~0) and stands tall.
    fb = ctx.part_world_aabb(frame)
    ctx.check(
        "stone frame rests on ground and stands tall",
        fb is not None and fb[0][2] >= -1e-3 and fb[1][2] > 2.3,
        details=f"frame_aabb={fb}",
    )

    # Closed door is upright (tall and thin), not lying flat.
    with ctx.pose({hinge: 0.0}):
        db = ctx.part_world_aabb(door)
        height = db[1][2] - db[0][2]
        thickness = db[1][1] - db[0][1]
        width = db[1][0] - db[0][0]
        ctx.check(
            "closed door is upright (height >> thickness)",
            db[0][2] >= -1e-3 and height > 1.8 and thickness < 0.25,
            details=f"door_aabb={db} h={height:.3f} t={thickness:.3f} w={width:.3f}",
        )
        # Closed door seats inside the opening width (between the jamb reveals).
        ctx.expect_within(
            door,
            frame,
            axes="z",
            margin=0.05,
            name="closed door fits under the arch",
        )
        closed_pos = ctx.part_world_position(door)

    # Open pose: the latch (free) edge swings clear toward the front (-Y),
    # proving the leaf actually opens on the vertical hinge.
    open_q = math.radians(100.0)
    with ctx.pose({hinge: open_q}):
        db_open = ctx.part_world_aabb(door)
        # When open ~100 deg, the leaf should extend well into -Y (in front of
        # the stone) and its X-extent should shrink toward the hinge line.
        ctx.check(
            "open door swings out in front of the frame",
            db_open[0][1] < -0.5,
            details=f"open_door_aabb={db_open}",
        )
        ctx.check(
            "open door rotates about the hinge edge (x near hinge shrinks)",
            db_open[1][0] < X_HINGE + 0.35,
            details=f"open_door_max_x={db_open[1][0]:.3f} hinge_x={X_HINGE:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
