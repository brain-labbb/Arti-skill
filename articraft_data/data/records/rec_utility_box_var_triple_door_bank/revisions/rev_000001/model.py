from __future__ import annotations

# Wide TRIPLE-DOOR street electrical cabinet on a stepped concrete pedestal.
#
# Reference: a roughly cubic roadside cabinet, wider than tall, weathered grey
# steel, standing on a stepped poured-concrete base. The front face is divided
# into N=3 equal side-by-side service doors, each hinged on its LEFT vertical
# edge and swinging outward independently. Each door carries its own louvered
# vent and recessed handle.
#
# Parametric door construction: ONE shared builder `_build_door` is called in a
# for-i-in-range(N) loop. Doors use door_{i} naming and regular X placement.
# All doors share the same revolute joint policy (vertical +Z axis, positive q
# opens the free edge forward / -Y).
#
# Coordinate convention (Z-up world):
#   +X : cabinet WIDTH  (~0.84 m)
#   +Y : cabinet DEPTH  (~0.55 m); front face at -Y (door side)
#   +Z : HEIGHT; concrete base at z=0, cabinet top ~0.97 m
#
# Parts / articulations:
#   - base (ROOT)    : stepped concrete pedestal, base at z=0
#   - body           : hollow grey-steel shell, open at the front
#   - door_0         : left door,   REVOLUTE about its left (+Z) edge
#   - door_1         : middle door, REVOLUTE about its left (+Z) edge
#   - door_2         : right door,  REVOLUTE about its left (+Z) edge

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
W = 0.84          # width (X)
D = 0.55          # depth (Y)
BODY_H = 0.72     # cabinet shell height above base
WALL = 0.012

# stepped concrete base: wide lower step + narrower upper step
BASE_LO_H = 0.10
BASE_HI_H = 0.12
BASE_H = BASE_LO_H + BASE_HI_H
BASE_LO_OVER = 0.12   # lower step overhang per side beyond body
BASE_HI_OVER = 0.05   # upper step overhang per side beyond body

# ---- door bank parameters ----
N_DOORS = 3
DOOR_GAP = 0.006        # reveal at outer edges (left and right)
SEAM_GAP = 0.005        # gap between adjacent doors
DOOR_T = 0.018
# each door spans equal width: total = W - 2*DOOR_GAP - (N-1)*SEAM_GAP
DOOR_W = (W - 2 * DOOR_GAP - (N_DOORS - 1) * SEAM_GAP) / N_DOORS
DOOR_H = BODY_H - 2 * DOOR_GAP

FRONT_Y = -D / 2.0


def _door_hinge_x(i: int) -> float:
    """World X of the left hinge edge for door i (0-indexed, left to right)."""
    return -W / 2.0 + DOOR_GAP + i * (DOOR_W + SEAM_GAP)


def _door_center_x(i: int) -> float:
    """World X of the centre of door i."""
    return _door_hinge_x(i) + DOOR_W / 2.0


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
        pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, -hy + wall / 2.0, 0.0))))
    return pieces


def _build_door(model, name, *, materials):
    """Author ONE parametric door.

    Local frame: origin at the hinge edge (left / -X side of the door as
    mounted). The skin extends toward local +X by DOOR_W, centred at
    (DOOR_W/2, 0, 0). Decorated face toward local -Y (forward).
    """
    steel_door, dark_metal, steel = materials
    front_face = -DOOR_T / 2.0
    skin_cx = DOOR_W / 2.0  # skin centre offset from hinge edge

    door = model.part(name)
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(skin_cx, 0.0, 0.0)),
        material=steel_door,
        name=f"{name}_skin",
    )
    # louvered ventilation slot in the upper-mid of the door face
    grille = SlotPatternPanelGeometry(
        (0.18, 0.10),
        0.006,
        slot_size=(0.12, 0.006),
        pitch=(0.18, 0.014),
        frame=0.012,
        corner_radius=0.004,
    )
    door.visual(
        mesh_from_geometry(grille, f"{name}_grille"),
        origin=Origin(xyz=(skin_cx, front_face - 0.001, DOOR_H / 2.0 - 0.18),
                      rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name=f"{name}_grille",
    )
    # recessed handle near the free (right) edge
    handle_x = DOOR_W - 0.04
    door.visual(
        Box((0.05, 0.006, 0.13)),
        origin=Origin(xyz=(handle_x, front_face - 0.003, 0.0)),
        material=dark_metal,
        name=f"{name}_handle_pocket",
    )
    door.visual(
        Box((0.013, 0.040, 0.06)),
        origin=Origin(xyz=(handle_x, front_face - 0.020, 0.0)),
        material=steel,
        name=f"{name}_handle_lever",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)), mass=5.0,
        origin=Origin(xyz=(skin_cx, 0.0, 0.0)),
    )
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_door_cabinet")

    steel = model.material("grey_steel", rgba=(0.64, 0.65, 0.67, 1.0))
    steel_door = model.material("door_steel", rgba=(0.71, 0.72, 0.74, 1.0))
    concrete = model.material("concrete", rgba=(0.56, 0.55, 0.52, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))

    # ---- base (ROOT) : stepped concrete pedestal, base at z=0 ----
    base = model.part("base")
    lo_w = W + 2 * BASE_LO_OVER
    lo_d = D + 2 * BASE_LO_OVER
    hi_w = W + 2 * BASE_HI_OVER
    hi_d = D + 2 * BASE_HI_OVER
    base.visual(
        Box((lo_w, lo_d, BASE_LO_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LO_H / 2.0)),
        material=concrete,
        name="base_lower_step",
    )
    base.visual(
        Box((hi_w, hi_d, BASE_HI_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LO_H + BASE_HI_H / 2.0)),
        material=concrete,
        name="base_upper_step",
    )
    base.inertial = Inertial.from_geometry(
        Box((lo_w, lo_d, BASE_H)), mass=80.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # ---- body : hollow grey-steel shell, open at the front ----
    body = model.part("body")
    body_cz = BASE_H + BODY_H / 2.0
    for i, (geom, org) in enumerate(_hollow_box((W, D, BODY_H), WALL, open_face="-y")):
        body.visual(
            geom,
            origin=Origin(xyz=(org.xyz[0], org.xyz[1], org.xyz[2] + body_cz)),
            material=steel,
            name=f"shell_{i}",
        )
    # mullions between adjacent doors (N-1 mullions at seam positions)
    for i in range(N_DOORS - 1):
        seam_x = _door_hinge_x(i) + DOOR_W + SEAM_GAP / 2.0
        body.visual(
            Box((SEAM_GAP + 0.010, WALL * 1.5, BODY_H)),
            origin=Origin(xyz=(seam_x, FRONT_Y + WALL, body_cz)),
            material=steel,
            name=f"mullion_{i}",
        )
    # top drip cap
    body.visual(
        Box((W + 0.04, D + 0.04, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BODY_H + 0.008)),
        material=steel_door,
        name="drip_cap",
    )
    # exposed barrel hinges: 2 per door, on each door's left jamb
    for i in range(N_DOORS):
        hx = _door_hinge_x(i)
        for j, hz in enumerate((BASE_H + 0.15, BASE_H + BODY_H - 0.15)):
            hinge = BarrelHingeGeometry(
                0.085,
                leaf_width_a=0.018,
                leaf_width_b=0.018,
                leaf_thickness=0.0024,
                pin_diameter=0.006,
                knuckle_count=5,
            )
            body.visual(
                mesh_from_geometry(hinge, f"hinge_{i}_{j}"),
                origin=Origin(xyz=(hx + 0.004, FRONT_Y + 0.013, hz)),
                material=dark_metal,
                name=f"hinge_{i}_{j}",
            )
    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)), mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, body_cz)),
    )

    # ---- N doors from the SAME parametric builder, regular X placement ----
    materials = (steel_door, dark_metal, steel)
    door_face_y = FRONT_Y + DOOR_T / 2.0 + DOOR_GAP
    door_cz = BASE_H + BODY_H / 2.0
    doors = []
    for i in range(N_DOORS):
        door_name = f"door_{i}"
        door = _build_door(model, door_name, materials=materials)
        doors.append(door)

        # Each door: hinge at its left edge, axis -Z so positive q swings
        # the free edge (+X from hinge) toward -Y (forward / outward).
        hinge_x = _door_hinge_x(i)
        model.articulation(
            f"body_to_{door_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, door_face_y, door_cz)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                lower=0.0, upper=math.radians(115.0), effort=22.0, velocity=2.0
            ),
        )

    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent=base,
        child=body,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    doors = [object_model.get_part(f"door_{i}") for i in range(N_DOORS)]
    joints = [object_model.get_articulation(f"body_to_door_{i}") for i in range(N_DOORS)]

    # ---- concrete base at z=0 ----
    bmn, _ = ctx.part_world_aabb(base)
    ctx.check("base at z=0", abs(bmn[2]) < 1e-3, details=f"base min z={bmn[2]:.4f}")

    # ---- all door joints are REVOLUTE about vertical -Z axis ----
    for i, joint in enumerate(joints):
        ax = joint.axis
        ctx.check(
            f"door_{i} hinge axis vertical (-Z)",
            abs(ax[2] + 1.0) < 1e-6 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
            details=f"axis={ax}",
        )
        ctx.check(
            f"door_{i} joint is revolute",
            str(joint.articulation_type).upper().endswith("REVOLUTE"),
            details=f"type={joint.articulation_type}",
        )

    # ---- N=3 doors exist ----
    ctx.check(
        "three doors present",
        len(doors) == 3 and all(d is not None for d in doors),
        details=f"door count={len(doors)}",
    )

    # ---- doors are regularly spaced across the front ----
    door_centers = []
    for d in doors:
        aabb = ctx.part_world_aabb(d)
        cx = (aabb[0][0] + aabb[1][0]) / 2.0
        door_centers.append(cx)
    # check roughly equal spacing
    if len(door_centers) >= 2:
        spacings = [door_centers[i + 1] - door_centers[i] for i in range(len(door_centers) - 1)]
        avg_spacing = sum(spacings) / len(spacings)
        ctx.check(
            "doors are regularly spaced in X",
            all(abs(s - avg_spacing) < 0.01 for s in spacings),
            details=f"spacings={spacings}",
        )

    # ---- all doors face forward (-Y) ----
    for i, d in enumerate(doors):
        aabb = ctx.part_world_aabb(d)
        ctx.check(
            f"door_{i} faces forward (-Y)",
            aabb[0][1] < FRONT_Y + 0.05,
            details=f"min_y={aabb[0][1]:.3f}",
        )

    # ---- each door opens independently (positive q swings free edge forward) ----
    for i, (d, j) in enumerate(zip(doors, joints)):
        closed_y = ctx.part_world_aabb(d)[0][1]
        with ctx.pose({j: math.radians(90.0)}):
            open_y = ctx.part_world_aabb(d)[0][1]
        ctx.check(
            f"door_{i} swings forward when opened",
            open_y < closed_y - 0.08,
            details=f"closed={closed_y:.3f}, open={open_y:.3f}",
        )

    # ---- each door carries its own louver vent ----
    for i in range(N_DOORS):
        d = doors[i]
        ctx.check(
            f"door_{i} has louver vent",
            d.get_visual(f"door_{i}_grille") is not None,
            details=f"door_{i}_grille present",
        )

    # ---- door independence: opening door_0 does not move door_2 ----
    d2_closed = ctx.part_world_aabb(doors[2])
    with ctx.pose({joints[0]: math.radians(90.0)}):
        d2_while_0_open = ctx.part_world_aabb(doors[2])
    ctx.check(
        "door_2 stays closed when door_0 opens (independent)",
        abs(d2_while_0_open[0][0] - d2_closed[0][0]) < 0.005
        and abs(d2_while_0_open[0][1] - d2_closed[0][1]) < 0.005,
        details=f"closed={d2_closed[0]}, while_0_open={d2_while_0_open[0]}",
    )

    # ---- closed doors nest in the opening ----
    for i in range(N_DOORS):
        ctx.allow_overlap(
            doors[i], body,
            reason=f"Closed door_{i} nests in the front opening; hinge knuckles meet the left jamb.",
        )

    return ctx.report()


object_model = build_object_model()
