from __future__ import annotations

# Tall, narrow street electrical utility cabinet (heavily ventilated variant).
#
# Reference: a slim free-standing roadside cabinet, clearly taller than wide,
# weathered white/grey paint heavily tagged with graffiti. A single front
# service door covers most of the front face, hinged on its LEFT vertical edge,
# with a recessed latch handle on the right and a TALL STACK of N=10 horizontal
# louvered ventilation rows running most of the door height. It stands on a
# short steel plinth/skirt that meets the ground at z=0.
#
# Coordinate convention (Z-up world):
#   +X : cabinet WIDTH  (~0.46 m across the front face)
#   +Y : cabinet DEPTH  (~0.30 m front-to-back); the front face is at -Y
#   +Z : HEIGHT; plinth base sits at z = 0, cabinet top ~0.92 m
#
# Parts / articulations:
#   - plinth (ROOT)  : short steel skirt base at z=0
#   - body           : tall hollow cabinet shell (enclosure), open at the front
#   - door           : single front panel, REVOLUTE about its LEFT vertical edge
#                      (axis +Z), N=10 louver rows + recessed handle, opens outward

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
W = 0.46          # width (X)
D = 0.30          # depth (Y)
BODY_H = 0.80     # height of the cabinet shell above the plinth
PLINTH_H = 0.10   # plinth skirt height
PLINTH_INSET = 0.015  # plinth slightly narrower than body footprint
WALL = 0.010      # shell wall thickness

DOOR_GAP = 0.006        # reveal gap around the door inside the front opening
DOOR_T = 0.018          # door panel thickness
DOOR_W = W - 2 * DOOR_GAP
DOOR_H = BODY_H - 2 * DOOR_GAP

FRONT_Y = -D / 2.0      # front face plane (door side)

# Door hinge is on the +X vertical edge; handle is at the -X free edge.
HINGE_X = +DOOR_W / 2.0

# ---- louver stack parameters ----
LOUVER_N = 10           # number of horizontal louver rows on the door face
LOUVER_W = 0.30         # louver row panel width
LOUVER_H = 0.022        # louver row panel height (narrow horizontal slot)
LOUVER_T = 0.005        # louver panel thickness
LOUVER_SLOT_W = 0.26    # slot opening width
LOUVER_SLOT_H = 0.010   # slot opening height
LOUVER_FRAME = 0.006    # frame border around the slot
LOUVER_MARGIN = 0.05    # top/bottom margin from door edges to first/last row


def _hollow_box(outer, wall, *, open_face):
    """Hollow rectangular shell as a list of (Box, Origin) wall pieces.

    outer = (sx, sy, sz) full extents, centred on local origin.
    open_face: face to leave open, '-y' (front) or None.
    """
    sx, sy, sz = outer
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    pieces = []
    pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, hy - wall / 2.0, 0.0))))  # back +Y
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(hx - wall / 2.0, 0.0, 0.0))))  # right +X
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(-hx + wall / 2.0, 0.0, 0.0))))  # left -X
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, hz - wall / 2.0))))  # top +Z
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, -hz + wall / 2.0))))  # bottom -Z
    if open_face != "-y":
        pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, -hy + wall / 2.0, 0.0))))
    return pieces


def _louver_row_mesh(name: str):
    """Shared geometry helper: one horizontal louver-row slot-pattern grille.

    Returns a managed mesh from a SlotPatternPanelGeometry sized for a single
    narrow horizontal louver slot, suitable for vertical stacking on a door.
    """
    grille = SlotPatternPanelGeometry(
        (LOUVER_W, LOUVER_H),
        LOUVER_T,
        slot_size=(LOUVER_SLOT_W, LOUVER_SLOT_H),
        pitch=(LOUVER_W, LOUVER_H),
        frame=LOUVER_FRAME,
        corner_radius=0.003,
    )
    return mesh_from_geometry(grille, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_cabinet_tall")

    paint = model.material("weathered_paint", rgba=(0.82, 0.81, 0.78, 1.0))
    paint_dark = model.material("door_paint", rgba=(0.74, 0.73, 0.70, 1.0))
    steel = model.material("galv_steel", rgba=(0.58, 0.59, 0.62, 1.0))
    vent_metal = model.material("vent_metal", rgba=(0.40, 0.41, 0.43, 1.0))

    # ---- plinth (ROOT) : short steel skirt, base at z=0 ----
    plinth = model.part("plinth")
    pw = W - 2 * PLINTH_INSET
    pd = D - 2 * PLINTH_INSET
    plinth.visual(
        Box((pw, pd, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
        material=steel,
        name="plinth_skirt",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((pw, pd, PLINTH_H)), mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
    )

    # ---- body : tall hollow cabinet shell, open at the front (-Y) ----
    body = model.part("body")
    body_cz = PLINTH_H + BODY_H / 2.0  # body centre Z (sits on top of plinth)
    for i, (geom, org) in enumerate(_hollow_box((W, D, BODY_H), WALL, open_face="-y")):
        body.visual(
            geom,
            origin=Origin(xyz=(org.xyz[0], org.xyz[1], org.xyz[2] + body_cz)),
            material=paint,
            name=f"shell_{i}",
        )
    # rain-cap drip lip overhanging the top edges
    body.visual(
        Box((W + 0.03, D + 0.03, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + BODY_H + 0.006)),
        material=paint_dark,
        name="rain_cap",
    )
    # two exposed barrel hinges on the +X jamb of the front opening
    for j, hz in enumerate((PLINTH_H + 0.16, PLINTH_H + BODY_H - 0.16)):
        hinge = BarrelHingeGeometry(
            0.080,
            leaf_width_a=0.018,
            leaf_width_b=0.018,
            leaf_thickness=0.0024,
            pin_diameter=0.006,
            knuckle_count=5,
        )
        # BarrelHinge pin axis is local Z -> vertical pin, no rotation needed.
        hm = mesh_from_geometry(hinge, f"hinge_{j}")
        body.visual(
            hm,
            origin=Origin(xyz=(HINGE_X - 0.004, FRONT_Y + 0.012, hz)),
            material=steel,
            name=f"hinge_{j}",
        )
    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)), mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, body_cz)),
    )

    # ---- door : single front panel, hinged on +X vertical edge ----
    # Authored so the LOCAL ORIGIN is the hinge edge: the door skin centre sits at
    # local x = -DOOR_W/2, so the hinge edge is local x=0 and the free (handle)
    # edge is at local x=-DOOR_W. This makes the hinge pivot coincide with the
    # joint origin at the +X jamb (door lies flush across the opening at q=0).
    door = model.part("door")
    skin_cx = -DOOR_W / 2.0
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(skin_cx, 0.0, 0.0)),
        material=paint_dark,
        name="door_skin",
    )
    # Tall stack of N horizontal louver rows evenly pitched up the door face.
    # SlotPatternPanelGeometry lies in local XY, extends along Z; a Visual rpy of
    # roll=+pi/2 maps panel-local +Z -> local -Y so the slotted face looks forward.
    louver_span = DOOR_H - 2 * LOUVER_MARGIN  # vertical span available for rows
    louver_pitch = louver_span / (LOUVER_N - 1) if LOUVER_N > 1 else 0.0
    for i in range(LOUVER_N):
        gz = -DOOR_H / 2.0 + LOUVER_MARGIN + i * louver_pitch
        lm = _louver_row_mesh(f"louver_{i}")
        door.visual(
            lm,
            origin=Origin(xyz=(skin_cx, -DOOR_T / 2.0 - 0.001, gz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=vent_metal,
            name=f"louver_{i}",
        )
    # recessed latch handle on the FREE edge (-X side, near local x=-DOOR_W)
    handle_x = -DOOR_W + 0.05
    door.visual(
        Box((0.06, 0.006, 0.12)),
        origin=Origin(xyz=(handle_x, -DOOR_T / 2.0 - 0.003, 0.0)),
        material=steel,
        name="handle_plate",
    )
    door.visual(
        Box((0.012, 0.040, 0.070)),
        origin=Origin(xyz=(handle_x, -DOOR_T / 2.0 - 0.020, 0.0)),
        material=steel,
        name="handle_lever",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)), mass=6.0,
        origin=Origin(xyz=(skin_cx, 0.0, 0.0)),
    )

    # ---- articulation : door REVOLUTE about its +X vertical edge ----
    # Joint origin at the +X hinge jamb on the front plane. The door's local
    # origin (its hinge edge) maps here at q=0, so the door lies flush across the
    # opening. The door extends toward -X; positive q about +Z swings the free
    # edge forward (-Y), opening outward.
    door_face_y = FRONT_Y + DOOR_T / 2.0 + DOOR_GAP  # door skin centre Y when closed
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, door_face_y, PLINTH_H + BODY_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.radians(115.0), effort=20.0, velocity=2.0
        ),
    )

    # ---- plinth -> body fixed joint ----
    model.articulation(
        "plinth_to_body",
        ArticulationType.FIXED,
        parent=plinth,
        child=body,
        origin=Origin(),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")

    # ---- base sits at z=0 (plinth bottom on the ground) ----
    pmn, _ = ctx.part_world_aabb(plinth)
    ctx.check("plinth base at z=0", abs(pmn[2]) < 1e-3, details=f"plinth min z={pmn[2]:.4f}")

    # ---- cabinet is clearly taller than wide (tall narrow form) ----
    bmn, bmx = ctx.part_world_aabb(body)
    full_h = bmx[2]
    ctx.check(
        "cabinet taller than wide",
        full_h > W + 0.25,
        details=f"height={full_h:.3f}, width={W:.3f}",
    )

    # ---- door joint is REVOLUTE about a vertical (+Z) axis ----
    ax = door_joint.axis
    ctx.check(
        "door hinge axis is vertical (Z)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
        details=f"axis={ax}",
    )
    ctx.check(
        "door joint is revolute",
        str(door_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={door_joint.articulation_type}",
    )

    # ---- door covers most of the front opening when closed ----
    de = _ext(ctx.part_world_aabb(door))
    ctx.check(
        "door spans most of cabinet height",
        de[2] > BODY_H * 0.85,
        details=f"door height extent={de[2]:.3f}",
    )

    # ---- door swings OUTWARD (toward -Y) about the +X jamb ----
    closed_aabb = ctx.part_world_aabb(door)
    closed_front_y = closed_aabb[0][1]
    closed_x_min = closed_aabb[0][0]
    # At 90deg the whole door swings forward (-Y) and pivots about the +X jamb,
    # so the door's -X extent collapses back toward the hinge while -Y grows.
    with ctx.pose({door_joint: math.radians(90.0)}):
        open_aabb = ctx.part_world_aabb(door)
        open_front_y = open_aabb[0][1]
        open_x_min = open_aabb[0][0]
    ctx.check(
        "door free edge swings out and forward when opened",
        open_front_y < closed_front_y - 0.10,
        details=f"closed_front_y={closed_front_y:.3f}, open_front_y={open_front_y:.3f}",
    )
    # Pivot is on the +X side: the free (-X) edge sweeps toward the hinge in X.
    ctx.check(
        "door rotates about the +X jamb (free edge sweeps in)",
        open_x_min > closed_x_min + 0.10,
        details=f"closed_x_min={closed_x_min:.3f}, open_x_min={open_x_min:.3f}",
    )

    # ---- tall stack of N louver rows present on the door ----
    louver_visuals = [door.get_visual(f"louver_{i}") for i in range(LOUVER_N)]
    ctx.check(
        f"door carries {LOUVER_N} louver rows",
        all(v is not None for v in louver_visuals),
        details=f"expected louver_0..louver_{LOUVER_N - 1} on door part",
    )
    # Louver rows are evenly pitched: top and bottom rows are symmetric about door centre.
    first_z = -DOOR_H / 2.0 + LOUVER_MARGIN
    last_z = DOOR_H / 2.0 - LOUVER_MARGIN
    ctx.check(
        "louver stack spans most of door height",
        (last_z - first_z) > DOOR_H * 0.70,
        details=f"stack span={last_z - first_z:.3f}, door_h={DOOR_H:.3f}",
    )

    # ---- closed door nests in the front opening; seam overlap allowed ----
    ctx.allow_overlap(
        door, body,
        reason="Closed door reveal nests in the front opening; hinge knuckles meet the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
