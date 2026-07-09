from __future__ import annotations

# Grey galvanized-steel street electrical distribution cabinet on a concrete plinth.
#
# Reference: a free-standing roadside distribution cabinet, brushed grey steel,
# sitting on a poured concrete pedestal that is wider than the body footprint.
# A single front service door spans the front face, hinged on a vertical edge,
# with a flush recessed handle/lock pocket on one side. The door carries a small
# rectangular warning label plate and a lightning-bolt hazard symbol, with a
# louvered ventilation slot near the bottom. Conduit cable stubs enter the base.
#
# Coordinate convention (Z-up world):
#   +X : cabinet WIDTH  (~0.62 m)
#   +Y : cabinet DEPTH  (~0.40 m); front face at -Y (door side)
#   +Z : HEIGHT; concrete plinth base at z=0, cabinet top ~1.05 m
#
# Parts / articulations:
#   - plinth (ROOT)  : concrete pedestal, base at z=0, wider than the body
#   - body           : hollow grey-steel cabinet shell, open at the front
#   - door           : single front panel REVOLUTE about its +X vertical edge,
#                      warning plate + lightning bolt + bottom louver + handle
#   Static detail on body: exposed barrel hinges, top drip cap, two conduit
#   stubs at the base.

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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
W = 0.62          # width (X)
D = 0.40          # depth (Y)
BODY_H = 0.85     # cabinet shell height above plinth
PLINTH_H = 0.20   # concrete pedestal height
PLINTH_OVER = 0.06  # plinth overhang beyond body footprint on each side
WALL = 0.012

DOOR_GAP = 0.006
DOOR_T = 0.020
DOOR_W = W - 2 * DOOR_GAP
DOOR_H = BODY_H - 2 * DOOR_GAP

FRONT_Y = -D / 2.0
HINGE_X = +DOOR_W / 2.0  # hinge on +X edge; handle on -X edge


def _hollow_box(outer, wall, *, open_face):
    sx, sy, sz = outer
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    pieces = []
    pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, hy - wall / 2.0, 0.0))))  # back
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(hx - wall / 2.0, 0.0, 0.0))))  # +X
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(-hx + wall / 2.0, 0.0, 0.0))))  # -X
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, hz - wall / 2.0))))  # top
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, -hz + wall / 2.0))))  # bottom
    if open_face != "-y":
        pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, -hy + wall / 2.0, 0.0))))
    return pieces


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="distribution_cabinet")

    steel = model.material("grey_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    steel_door = model.material("door_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    concrete = model.material("concrete", rgba=(0.55, 0.54, 0.51, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))
    label_white = model.material("label_plate", rgba=(0.90, 0.90, 0.88, 1.0))
    hazard_yellow = model.material("hazard_yellow", rgba=(0.92, 0.78, 0.10, 1.0))
    bolt_red = model.material("bolt_red", rgba=(0.80, 0.12, 0.10, 1.0))

    # ---- plinth (ROOT) : concrete pedestal, base at z=0 ----
    plinth = model.part("plinth")
    pw = W + 2 * PLINTH_OVER
    pd = D + 2 * PLINTH_OVER
    plinth.visual(
        Box((pw, pd, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
        material=concrete,
        name="concrete_pedestal",
    )
    # two conduit stubs emerging from the pedestal top into the cabinet underside
    for c, cx in enumerate((-0.14, 0.14)):
        plinth.visual(
            Cylinder(radius=0.022, length=0.06),
            origin=Origin(xyz=(cx, 0.08, PLINTH_H + 0.02)),
            material=dark_metal,
            name=f"conduit_{c}",
        )
    plinth.inertial = Inertial.from_geometry(
        Box((pw, pd, PLINTH_H)), mass=60.0,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
    )

    # ---- body : hollow grey-steel shell, open at the front ----
    body = model.part("body")
    body_cz = PLINTH_H + BODY_H / 2.0
    for i, (geom, org) in enumerate(_hollow_box((W, D, BODY_H), WALL, open_face="-y")):
        body.visual(
            geom,
            origin=Origin(xyz=(org.xyz[0], org.xyz[1], org.xyz[2] + body_cz)),
            material=steel,
            name=f"shell_{i}",
        )
    # sloped-look drip cap at the top
    body.visual(
        Box((W + 0.04, D + 0.04, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + BODY_H + 0.008)),
        material=steel_door,
        name="drip_cap",
    )
    # exposed barrel hinges on the +X jamb
    for j, hz in enumerate((PLINTH_H + 0.16, PLINTH_H + BODY_H - 0.16)):
        hinge = BarrelHingeGeometry(
            0.090,
            leaf_width_a=0.020,
            leaf_width_b=0.020,
            leaf_thickness=0.0026,
            pin_diameter=0.007,
            knuckle_count=5,
        )
        body.visual(
            mesh_from_geometry(hinge, f"hinge_{j}"),
            origin=Origin(xyz=(HINGE_X - 0.004, FRONT_Y + 0.013, hz)),
            material=dark_metal,
            name=f"hinge_{j}",
        )
    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)), mass=40.0,
        origin=Origin(xyz=(0.0, 0.0, body_cz)),
    )

    # ---- door : single front panel, hinged on +X edge ----
    # Authored so the LOCAL ORIGIN is the +X hinge edge: features are offset by
    # CX = -DOOR_W/2 so the door skin centre sits at local x=-DOOR_W/2, the hinge
    # edge at local x=0, and the free (handle) edge at local x=-DOOR_W. This makes
    # the pivot coincide with the joint origin so the door lies flush at q=0.
    door = model.part("door")
    CX = -DOOR_W / 2.0
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(CX, 0.0, 0.0)),
        material=steel_door,
        name="door_skin",
    )
    front_face = -DOOR_T / 2.0  # local -Y face of the door

    # warning label plate near the top
    door.visual(
        Box((0.16, 0.004, 0.05)),
        origin=Origin(xyz=(CX, front_face - 0.002, DOOR_H / 2.0 - 0.12)),
        material=label_white,
        name="warning_label",
    )
    # lightning-bolt hazard: yellow triangle plate + red bolt, mid-upper face
    bolt_z = 0.06
    door.visual(
        Box((0.10, 0.004, 0.10)),
        origin=Origin(xyz=(CX, front_face - 0.002, bolt_z)),
        material=hazard_yellow,
        name="hazard_triangle",
    )
    # the lightning bolt: two short angled bars forming a Z, standing proud
    door.visual(
        Box((0.012, 0.006, 0.05)),
        origin=Origin(xyz=(CX - 0.005, front_face - 0.005, bolt_z + 0.015),
                      rpy=(0.0, math.radians(28.0), 0.0)),
        material=bolt_red,
        name="bolt_upper",
    )
    door.visual(
        Box((0.012, 0.006, 0.05)),
        origin=Origin(xyz=(CX + 0.005, front_face - 0.005, bolt_z - 0.015),
                      rpy=(0.0, math.radians(28.0), 0.0)),
        material=bolt_red,
        name="bolt_lower",
    )
    # louvered ventilation slot near the bottom of the door
    grille = SlotPatternPanelGeometry(
        (0.30, 0.10),
        0.006,
        slot_size=(0.24, 0.006),
        pitch=(0.30, 0.014),
        frame=0.014,
        corner_radius=0.004,
    )
    door.visual(
        mesh_from_geometry(grille, "door_grille"),
        origin=Origin(xyz=(CX, front_face - 0.001, -DOOR_H / 2.0 + 0.12),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="door_grille",
    )
    # flush recessed handle/lock pocket on the FREE (-X) edge (near local -DOOR_W)
    handle_x = -DOOR_W + 0.05
    door.visual(
        Box((0.05, 0.006, 0.14)),
        origin=Origin(xyz=(handle_x, front_face - 0.003, 0.0)),
        material=dark_metal,
        name="handle_pocket",
    )
    door.visual(
        Box((0.014, 0.045, 0.06)),
        origin=Origin(xyz=(handle_x, front_face - 0.022, 0.0)),
        material=steel,
        name="handle_lever",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)), mass=9.0,
        origin=Origin(xyz=(CX, 0.0, 0.0)),
    )

    # ---- articulation : door REVOLUTE about its +X vertical edge ----
    door_face_y = FRONT_Y + DOOR_T / 2.0 + DOOR_GAP
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, door_face_y, PLINTH_H + BODY_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.radians(120.0), effort=24.0, velocity=2.0
        ),
    )
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

    # ---- concrete plinth base at z=0 and wider than body ----
    pmn, pmx = ctx.part_world_aabb(plinth)
    ctx.check("plinth base at z=0", abs(pmn[2]) < 1e-3, details=f"plinth min z={pmn[2]:.4f}")
    bmn, bmx = ctx.part_world_aabb(body)
    ctx.check(
        "plinth footprint wider than body",
        (pmx[0] - pmn[0]) > (bmx[0] - bmn[0]) + 0.05,
        details=f"plinth_x={(pmx[0]-pmn[0]):.3f}, body_x={(bmx[0]-bmn[0]):.3f}",
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

    # ---- door covers most of the front opening ----
    de = _ext(ctx.part_world_aabb(door))
    ctx.check(
        "door spans most of cabinet height",
        de[2] > BODY_H * 0.85,
        details=f"door height extent={de[2]:.3f}",
    )

    # ---- door swings forward (-Y) about the +X jamb ----
    closed_aabb = ctx.part_world_aabb(door)
    closed_front_y = closed_aabb[0][1]
    closed_x_min = closed_aabb[0][0]
    with ctx.pose({door_joint: math.radians(90.0)}):
        open_aabb = ctx.part_world_aabb(door)
        open_front_y = open_aabb[0][1]
        open_x_min = open_aabb[0][0]
    ctx.check(
        "door swings forward when opened",
        open_front_y < closed_front_y - 0.10,
        details=f"closed_front_y={closed_front_y:.3f}, open_front_y={open_front_y:.3f}",
    )
    ctx.check(
        "door pivots on the +X jamb (free edge sweeps in)",
        open_x_min > closed_x_min + 0.10,
        details=f"closed_x_min={closed_x_min:.3f}, open_x_min={open_x_min:.3f}",
    )

    # ---- warning plate, hazard symbol, and louver present ----
    for vis_name in ("warning_label", "hazard_triangle", "bolt_upper", "door_grille"):
        ctx.check(
            f"{vis_name} present on door",
            door.get_visual(vis_name) is not None,
            details=f"door exposes {vis_name}",
        )

    # ---- closed door nests in the front opening; seam overlap allowed ----
    ctx.allow_overlap(
        door, body,
        reason="Closed door reveal nests in the front opening; hinge knuckles meet the jamb.",
    )

    # ---- conduit stubs intentionally penetrate the cabinet floor (cable entry) ----
    for c in (0, 1):
        ctx.allow_overlap(
            plinth, body,
            elem_a=f"conduit_{c}",
            elem_b="shell_4",
            reason="Conduit cable stub enters the cabinet through its bottom wall.",
        )
    ctx.expect_overlap(
        plinth, body,
        axes="xy",
        elem_a="conduit_0",
        elem_b="shell_4",
        min_overlap=0.01,
        name="conduit enters cabinet floor footprint",
    )

    return ctx.report()


object_model = build_object_model()
