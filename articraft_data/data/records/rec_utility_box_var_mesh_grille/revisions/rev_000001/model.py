from __future__ import annotations

# Tall, narrow street electrical utility cabinet (graffiti-covered painted steel).
# Mesh-grille variant: perforated ventilation panel on the door face.
#
# Reference: a slim free-standing roadside cabinet, clearly taller than wide,
# weathered white/grey paint heavily tagged with graffiti. A single front
# service door covers most of the front face, hinged on its LEFT vertical edge,
# with a recessed latch handle on the right and a perforated mesh ventilation
# grille panel (regular grid of small round holes) across most of the door.
# It stands on a short steel plinth/skirt that meets the ground at z=0.
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
#                      (axis +Z), perforated mesh grille + recessed handle

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
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

# ---- perforated mesh grille parameters ----
MESH_HOLE_RADIUS = 0.005      # 5 mm radius (10 mm diameter round holes)
MESH_HOLE_LENGTH = 0.003      # 3 mm thick cylinder stand-ins for through-holes
MESH_GRID_W = 0.28            # perforated area width (door-local X)
MESH_GRID_H = 0.52            # perforated area height (door-local Z)
MESH_PITCH = 0.035            # 35 mm regular spacing between hole centres
MESH_FRAME_W = 0.010          # frame border width around the perforated area
MESH_COLS = int(MESH_GRID_W / MESH_PITCH)
MESH_ROWS = int(MESH_GRID_H / MESH_PITCH)
MESH_HOLE_COUNT = MESH_COLS * MESH_ROWS


def _mesh_hole_geometry():
    """Shared geometry descriptor for one perforated mesh hole cylinder."""
    return Cylinder(radius=MESH_HOLE_RADIUS, length=MESH_HOLE_LENGTH)


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_cabinet_tall")

    paint = model.material("weathered_paint", rgba=(0.82, 0.81, 0.78, 1.0))
    paint_dark = model.material("door_paint", rgba=(0.74, 0.73, 0.70, 1.0))
    steel = model.material("galv_steel", rgba=(0.58, 0.59, 0.62, 1.0))
    vent_metal = model.material("vent_metal", rgba=(0.40, 0.41, 0.43, 1.0))
    hole_dark = model.material("hole_dark", rgba=(0.06, 0.06, 0.07, 1.0))

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
    # Perforated mesh ventilation grille panel covering most of the door face.
    # A regular grid of small round through-holes (dark cylinders) on the
    # front (-Y) face, with a thin metal frame border around the perforated area.
    mesh_cols = MESH_COLS
    mesh_rows = MESH_ROWS

    # Frame border around the perforated area (4 thin bars forming a rectangle)
    frame_y = -DOOR_T / 2.0 - 0.001  # slightly proud of door face
    fw = MESH_GRID_W + 2 * MESH_FRAME_W
    fh = MESH_GRID_H + 2 * MESH_FRAME_W
    # top bar
    door.visual(
        Box((fw, 0.003, MESH_FRAME_W)),
        origin=Origin(xyz=(skin_cx, frame_y, MESH_GRID_H / 2.0 + MESH_FRAME_W / 2.0)),
        material=vent_metal, name="mesh_frame_top",
    )
    # bottom bar
    door.visual(
        Box((fw, 0.003, MESH_FRAME_W)),
        origin=Origin(xyz=(skin_cx, frame_y, -MESH_GRID_H / 2.0 - MESH_FRAME_W / 2.0)),
        material=vent_metal, name="mesh_frame_bottom",
    )
    # left bar (toward hinge, +X side of door-local)
    door.visual(
        Box((MESH_FRAME_W, 0.003, MESH_GRID_H)),
        origin=Origin(xyz=(skin_cx + MESH_GRID_W / 2.0 + MESH_FRAME_W / 2.0, frame_y, 0.0)),
        material=vent_metal, name="mesh_frame_left",
    )
    # right bar (toward handle, -X side of door-local)
    door.visual(
        Box((MESH_FRAME_W, 0.003, MESH_GRID_H)),
        origin=Origin(xyz=(skin_cx - MESH_GRID_W / 2.0 - MESH_FRAME_W / 2.0, frame_y, 0.0)),
        material=vent_metal, name="mesh_frame_right",
    )

    # Mesh hole grid: for-i-in-range loop over rows and columns
    hole_geom = _mesh_hole_geometry()
    x_origin = skin_cx - MESH_GRID_W / 2.0 + MESH_PITCH / 2.0
    z_origin = -MESH_GRID_H / 2.0 + MESH_PITCH / 2.0
    i = 0
    for row in range(mesh_rows):
        for col in range(mesh_cols):
            hx = x_origin + col * MESH_PITCH
            hz = z_origin + row * MESH_PITCH
            door.visual(
                hole_geom,
                origin=Origin(
                    xyz=(hx, -DOOR_T / 2.0, hz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=hole_dark,
                name=f"mesh_hole_{i}",
            )
            i += 1
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

    # ---- perforated mesh grille present on the door ----
    h0 = door.get_visual("mesh_hole_0")
    h_last = door.get_visual(f"mesh_hole_{MESH_HOLE_COUNT - 1}")
    ctx.check(
        "perforated mesh grille holes on the door",
        h0 is not None and h_last is not None,
        details=f"door exposes mesh_hole_0 through mesh_hole_{MESH_HOLE_COUNT - 1}",
    )

    # ---- mesh grille covers significant portion of door face ----
    found_holes = sum(
        1 for v in door.visuals if v.name.startswith("mesh_hole_")
    )
    ctx.check(
        "mesh grille has full grid of holes",
        found_holes == MESH_HOLE_COUNT,
        details=f"expected {MESH_HOLE_COUNT} holes, found {found_holes}",
    )

    # ---- mesh frame present ----
    frame_parts = ["mesh_frame_top", "mesh_frame_bottom", "mesh_frame_left", "mesh_frame_right"]
    frame_ok = all(door.get_visual(fp) is not None for fp in frame_parts)
    ctx.check(
        "mesh panel frame borders present",
        frame_ok,
        details=f"door exposes {', '.join(frame_parts)}",
    )

    # ---- closed door nests in the front opening; seam overlap allowed ----
    ctx.allow_overlap(
        door, body,
        reason="Closed door reveal nests in the front opening; hinge knuckles meet the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
