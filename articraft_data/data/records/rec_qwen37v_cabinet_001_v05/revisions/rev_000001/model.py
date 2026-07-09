from __future__ import annotations

"""Variant 05: Industrial storage cabinet with glass-framed upper doors, solid
lower doors, tambour sliding front, visible hinge barrels, and caster feet.

Overall ~1.6 m wide x 0.5 m deep x 1.8 m tall. Brushed/tarnished steel body.
The front is divided into three horizontal zones:
  - Upper zone: two glass-framed hinged doors (left hinges left, right hinges
    right), each a steel frame with a semi-transparent glass panel.
  - Middle zone: open shelf with a tambour panel that slides sideways on a
    prismatic joint (X axis).
  - Lower zone: two solid steel hinged doors with vent slots and latch knobs.
All four doors have clearly visible hinge barrels on the hinge side. Four
caster blocks with wheel representations sit at the base.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60
CAB_D = 0.50
CAB_TOP = 1.80
CASTER_H = 0.10
WALL_T = 0.02

FRONT_Y = CAB_D / 2.0  # +0.25
BACK_Y = -CAB_D / 2.0

# Horizontal zone boundaries (Z)
LOWER_BOT = CASTER_H + 0.04  # 0.14  bottom rail top
LOWER_TOP = 0.68  # lower doors top / middle rail bottom
MID_BOT = 0.72  # tambour opening bottom
MID_TOP = 1.06  # tambour opening top / upper rail bottom
UPPER_BOT = 1.10  # upper doors bottom
UPPER_TOP = 1.74  # upper doors top / top rail bottom

# Derived zone heights
LOWER_DOOR_H = LOWER_TOP - LOWER_BOT  # 0.54
UPPER_DOOR_H = UPPER_TOP - UPPER_BOT  # 0.64
TAMBOUR_OPENING_H = MID_TOP - MID_BOT  # 0.34

LOWER_DOOR_ZC = 0.5 * (LOWER_BOT + LOWER_TOP)  # 0.41
UPPER_DOOR_ZC = 0.5 * (UPPER_BOT + UPPER_TOP)  # 1.42
MID_ZC = 0.5 * (MID_BOT + MID_TOP)  # 0.89

DOOR_T = WALL_T  # 0.02

# Two doors per section, centre stile at x=0
DOOR_W = 0.755
CENTER_STILE_W = 0.03

# Hinge x positions
LEFT_HINGE_X = -(CAB_W / 2.0 - WALL_T) + 0.001  # -0.779
RIGHT_HINGE_X = (CAB_W / 2.0 - WALL_T) - 0.001  # +0.779

# Visible hinge barrel dimensions
BARREL_R = 0.011
KNUCKLE_R = 0.014
BARREL_UNIT_H = 0.055

# Tambour
TAMBOUR_W = 0.82
TAMBOUR_H = TAMBOUR_OPENING_H - 0.01  # 0.33
TAMBOUR_T = 0.015
TAMBOUR_SLIDE = 0.76
# Tambour sits flush with the front face (centre stile is split, no obstruction)
TAMBOUR_Y_BACK = FRONT_Y - TAMBOUR_T  # back face at front plane minus thickness

# Latch
KNOB_TURN = math.radians(90.0)
DOOR_OPEN = math.radians(110.0)

# Top cap
CAP_T = 0.022
CAP_OVERHANG = 0.02

# Glass frame
FRAME_W = 0.040
GLASS_T = 0.004

# Vent slot (lower doors) - sized to stay within the door leaf
SLOT_LEN = 0.22
SLOT_W = 0.028
SLOT_ZC_LOCAL = -0.10  # in door-local z (door centre = 0)

# Caster
CASTER_BLOCK_SIZE = 0.055


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------


def _glass_door_frame_solid(sign: float, mesh_name: str):
    """Glass door frame: flat rectangular border with a centre opening.
    ``sign``=+1: panel extends +X from hinge; -1: extends -X."""
    xc = sign * DOOR_W / 2.0
    outer = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, UPPER_DOOR_H)
        .translate((xc, 0.0, 0.0))
    )
    iw = DOOR_W - 2.0 * FRAME_W
    ih = UPPER_DOOR_H - 2.0 * FRAME_W
    cutter = (
        cq.Workplane("XY")
        .box(iw, DOOR_T + 0.01, ih)
        .translate((xc, 0.0, 0.0))
    )
    frame = outer.cut(cutter)
    return mesh_from_cadquery(frame, mesh_name)


def _solid_door_solid(sign: float, mesh_name: str):
    """Solid lower door leaf with a rounded-end through vent slot near the
    bottom. ``sign``=+1 extends +X, -1 extends -X."""
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, LOWER_DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC_LOCAL))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _hinge_barrel_solid(barrel_len: float, mesh_name: str):
    """Three discrete visible barrel hinges along the door hinge edge."""
    result = None
    positions = [-barrel_len / 3.0, 0.0, barrel_len / 3.0]
    for zc in positions:
        unit = (
            cq.Workplane("XY")
            .circle(BARREL_R)
            .extrude(BARREL_UNIT_H / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        knuckle = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(0.018 / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        unit = unit.union(knuckle)
        result = unit if result is None else result.union(unit)
    return mesh_from_cadquery(result, mesh_name)


def _caster_solid(mesh_name: str):
    """Caster block: mounting plate + fork + wheel."""
    block = (
        cq.Workplane("XY")
        .box(CASTER_BLOCK_SIZE, CASTER_BLOCK_SIZE, CASTER_H - 0.025)
        .translate((0.0, 0.0, (CASTER_H - 0.025) / 2.0 + 0.025))
    )
    wheel = (
        cq.Workplane("YZ")
        .circle(0.018)
        .extrude(0.028 / 2.0, both=True)
        .translate((0.0, 0.0, 0.018))
    )
    for sx in (-1.0, 1.0):
        fork = (
            cq.Workplane("XY")
            .box(0.005, 0.040, 0.035)
            .translate((sx * 0.018, 0.0, 0.035))
        )
        block = block.union(fork)
    caster = block.union(wheel)
    return mesh_from_cadquery(caster, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_storage_cabinet_v05")

    # -- Materials ----------------------------------------------------------
    steel_body = model.material("steel_body", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_door = model.material("steel_door", rgba=(0.53, 0.54, 0.56, 1.0))
    steel_frame = model.material("steel_frame", rgba=(0.48, 0.49, 0.52, 1.0))
    glass_mat = model.material("glass", rgba=(0.72, 0.80, 0.86, 0.35))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.44, 0.45, 0.47, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_caster = model.material("steel_caster", rgba=(0.35, 0.36, 0.38, 1.0))
    tambour_mat = model.material("tambour", rgba=(0.50, 0.51, 0.53, 1.0))

    # =====================================================================
    # CABINET BODY
    # =====================================================================
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - CASTER_H  # 1.70
    carcass_zc = CASTER_H + carcass_h / 2.0  # 0.95

    # Side walls
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CASTER_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )

    # Interior shelf (in the middle tambour zone)
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, CAB_D - 2.0 * WALL_T, 0.015)),
        origin=Origin(xyz=(0.0, -0.01, MID_BOT + 0.008)),
        material=steel_body,
        name="mid_shelf",
    )

    # Front frame: horizontal rails (exact fill, no extension into door zones)
    # Bottom rail: CASTER_H to LOWER_BOT
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, LOWER_BOT - CASTER_H)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (CASTER_H + LOWER_BOT) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    # Middle rail: LOWER_TOP to MID_BOT
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, MID_BOT - LOWER_TOP)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LOWER_TOP + MID_BOT) / 2.0)
        ),
        material=steel_body,
        name="front_mid_rail",
    )
    # Upper rail: MID_TOP to UPPER_BOT
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, UPPER_BOT - MID_TOP)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (MID_TOP + UPPER_BOT) / 2.0)
        ),
        material=steel_body,
        name="front_upper_rail",
    )
    # Top rail: UPPER_TOP to CAB_TOP
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - UPPER_TOP)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (UPPER_TOP + CAB_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_top_rail",
    )

    # Centre stile split: lower portion (lower door zone) and upper portion
    # (upper door zone). The middle zone is an open shelf for the tambour.
    lower_stile_h = LOWER_TOP - LOWER_BOT
    body.visual(
        Box((CENTER_STILE_W, WALL_T, lower_stile_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LOWER_BOT + LOWER_TOP) / 2.0)
        ),
        material=steel_trim,
        name="centre_stile_lower",
    )
    upper_stile_h = UPPER_TOP - UPPER_BOT
    body.visual(
        Box((CENTER_STILE_W, WALL_T, upper_stile_h)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (UPPER_BOT + UPPER_TOP) / 2.0)
        ),
        material=steel_trim,
        name="centre_stile_upper",
    )

    # Tambour track rails: overlap with adjacent front rails for connectivity.
    # Lower track sits at the top of the mid rail, overlapping into it.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T + 0.004, 0.008)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0 + 0.002, MID_BOT - 0.004)
        ),
        material=steel_trim,
        name="tambour_track_lower",
    )
    # Upper track sits at the bottom of the upper rail.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T + 0.004, 0.008)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0 + 0.002, MID_TOP + 0.004)
        ),
        material=steel_trim,
        name="tambour_track_upper",
    )

    # Riveted top cap
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Rivet dots along the top rail
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Caster blocks at the four corners
    caster_mesh = _caster_solid("caster")
    caster_corners = [
        (CAB_W / 2.0 - 0.08, CAB_D / 2.0 - 0.08),
        (-CAB_W / 2.0 + 0.08, CAB_D / 2.0 - 0.08),
        (-CAB_W / 2.0 + 0.08, -CAB_D / 2.0 + 0.08),
        (CAB_W / 2.0 - 0.08, -CAB_D / 2.0 + 0.08),
    ]
    for i, (cx, cy) in enumerate(caster_corners):
        body.visual(
            caster_mesh,
            origin=Origin(xyz=(cx, cy, 0.0)),
            material=steel_caster,
            name=f"caster_{i}",
        )

    # =====================================================================
    # UPPER GLASS DOORS (2)
    # =====================================================================
    upper_hinge_barrel_len = UPPER_DOOR_H - 0.06
    glass_iw = DOOR_W - 2.0 * FRAME_W
    glass_ih = UPPER_DOOR_H - 2.0 * FRAME_W

    upper_door_specs = [
        (LEFT_HINGE_X, +1.0),  # left door, hinges left
        (RIGHT_HINGE_X, -1.0),  # right door, hinges right
    ]
    for i, (hinge_x, sign) in enumerate(upper_door_specs):
        door = model.part(f"upper_door_{i}")
        xc = sign * DOOR_W / 2.0

        # Steel frame
        door.visual(
            _glass_door_frame_solid(sign, f"upper_frame_{i}"),
            material=steel_frame,
            name="frame",
        )
        # Glass panel in the frame opening
        door.visual(
            Box((glass_iw, GLASS_T, glass_ih)),
            origin=Origin(xyz=(xc, 0.0, 0.0)),
            material=glass_mat,
            name="glass_panel",
        )
        # Visible hinge barrels
        door.visual(
            _hinge_barrel_solid(upper_hinge_barrel_len, f"upper_hinge_barrel_{i}"),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        model.articulation(
            f"upper_door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, UPPER_DOOR_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=30.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )

        # Small latch at mid-height near free edge.
        # The frame front face is at y = +DOOR_T/2 in door-local coords.
        knob = model.part(f"upper_latch_{i}")
        knob.visual(
            Cylinder(radius=0.015, length=0.005),
            origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="backplate",
        )
        knob.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="boss",
        )
        model.articulation(
            f"upper_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            # Mount on the frame border near the free edge, front face
            origin=Origin(
                xyz=(sign * (DOOR_W - FRAME_W / 2.0), DOOR_T / 2.0, 0.0)
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    # =====================================================================
    # LOWER SOLID DOORS (2)
    # =====================================================================
    lower_hinge_barrel_len = LOWER_DOOR_H - 0.06

    lower_door_specs = [
        (LEFT_HINGE_X, +1.0),  # left door
        (RIGHT_HINGE_X, -1.0),  # right door
    ]
    for i, (hinge_x, sign) in enumerate(lower_door_specs):
        door = model.part(f"lower_door_{i}")
        xc = sign * DOOR_W / 2.0

        # Solid panel with vent slot
        door.visual(
            _solid_door_solid(sign, f"lower_leaf_{i}"),
            material=steel_door,
            name="leaf",
        )
        # Dark backing behind vent slot
        backing_h = SLOT_LEN + 0.030
        door.visual(
            Box((SLOT_W + 0.014, 0.005, backing_h)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC_LOCAL)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near the top
        for j, dz in enumerate((0.18, 0.20, 0.22)):
            door.visual(
                Box((0.15, 0.004, 0.005)),
                origin=Origin(xyz=(xc, -0.001, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Visible hinge barrels
        door.visual(
            _hinge_barrel_solid(lower_hinge_barrel_len, f"lower_hinge_barrel_{i}"),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        model.articulation(
            f"lower_door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, LOWER_DOOR_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )

        # Quarter-turn latch knob
        knob = model.part(f"lower_latch_{i}")
        knob.visual(
            Cylinder(radius=0.018, length=0.005),
            origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="backplate",
        )
        knob.visual(
            Cylinder(radius=0.0065, length=0.014),
            origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_knob,
            name="boss",
        )
        knob.visual(
            Box((0.010, 0.008, 0.034)),
            origin=Origin(xyz=(0.0, 0.020, 0.0)),
            material=steel_knob,
            name="handle_bar",
        )
        knob.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(0.0, 0.020, -0.019)),
            material=steel_knob,
            name="handle_tip",
        )
        model.articulation(
            f"lower_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            origin=Origin(xyz=(sign * (DOOR_W - 0.10), 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    # =====================================================================
    # TAMBOUR SLIDING FRONT
    # =====================================================================
    tambour = model.part("tambour")

    # Main panel
    tambour.visual(
        Box((TAMBOUR_W, TAMBOUR_T, TAMBOUR_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tambour_mat,
        name="panel",
    )
    # Horizontal slat lines on the front face
    n_slats = 10
    for j in range(n_slats):
        dz = -TAMBOUR_H / 2.0 + 0.015 + j * ((TAMBOUR_H - 0.03) / (n_slats - 1))
        tambour.visual(
            Box((TAMBOUR_W - 0.02, 0.003, 0.003)),
            origin=Origin(xyz=(0.0, TAMBOUR_T / 2.0 + 0.001, dz)),
            material=steel_dark,
            name=f"slat_{j}",
        )
    # Top and bottom edge strips (slide in the tracks) - overlap panel slightly
    for edge_z, ename in (
        (TAMBOUR_H / 2.0 + 0.001, "top_edge"),
        (-TAMBOUR_H / 2.0 - 0.001, "bottom_edge"),
    ):
        tambour.visual(
            Box((TAMBOUR_W + 0.01, 0.020, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, edge_z)),
            material=steel_trim,
            name=ename,
        )

    # Prismatic joint: slides along X axis.
    # Tambour back face at FRONT_Y - TAMBOUR_T, flush with front frame plane.
    tambour_y_center = FRONT_Y - TAMBOUR_T / 2.0
    tambour_rest_x = -0.78 + TAMBOUR_W / 2.0  # -0.37
    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(tambour_rest_x, tambour_y_center, MID_ZC)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=TAMBOUR_SLIDE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    upper_doors = [object_model.get_part(f"upper_door_{i}") for i in range(2)]
    lower_doors = [object_model.get_part(f"lower_door_{i}") for i in range(2)]
    upper_hinges = [
        object_model.get_articulation(f"upper_door_{i}_hinge") for i in range(2)
    ]
    lower_hinges = [
        object_model.get_articulation(f"lower_door_{i}_hinge") for i in range(2)
    ]
    tambour = object_model.get_part("tambour")
    tambour_joint = object_model.get_articulation("tambour_slide")

    # --- Allowances: hinge barrels lap the frame edges ---------------------
    for door, frame_elem in zip(
        upper_doors + lower_doors,
        ["side_wall_0", "side_wall_1", "side_wall_0", "side_wall_1"],
    ):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=frame_elem,
            reason="Visible hinge barrel intentionally wraps the frame edge it pivots on.",
        )
        # Proof: hinge barrel contacts the frame
        ctx.expect_contact(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=frame_elem,
            contact_tol=0.006,
            name=f"{frame_elem} hinge barrel contacts frame edge",
        )

    # --- Overall envelope --------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m",
            1.58 <= (x1 - x0) <= 1.70,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.48 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.8 m",
            1.78 <= z1 <= 1.88,
            details=f"top={z1:.3f}",
        )
        ctx.check(
            "casters rest on the floor",
            abs(z0) <= 0.001,
            details=f"zmin={z0:.5f}",
        )

    # --- Upper glass doors -------------------------------------------------
    for i, (door, hinge) in enumerate(zip(upper_doors, upper_hinges)):
        ctx.check(
            f"upper_door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"upper_door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        gaabb = ctx.part_element_world_aabb(door, elem="glass_panel")
        ctx.check(
            f"upper_door_{i} has glass panel",
            gaabb is not None,
            details=str(gaabb),
        )
        haabb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"upper_door_{i} has visible hinge barrel",
            haabb is not None and (haabb[1][2] - haabb[0][2]) > 0.20,
            details=str(haabb),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"upper_door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DOOR_OPEN) < 1e-6,
        )

    # --- Lower solid doors -------------------------------------------------
    for i, (door, hinge) in enumerate(zip(lower_doors, lower_hinges)):
        ctx.check(
            f"lower_door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"lower_door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        laabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"lower_door_{i} has solid leaf",
            laabb is not None,
            details=str(laabb),
        )
        # Vent backing is in the lower portion of the door
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"lower_door_{i} has vent slot backing in lower half",
            vb is not None
            and 0.5 * (vb[0][2] + vb[1][2]) < LOWER_DOOR_ZC,
            details=str(vb),
        )
        haabb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"lower_door_{i} has visible hinge barrel",
            haabb is not None and (haabb[1][2] - haabb[0][2]) > 0.15,
            details=str(haabb),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"lower_door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - DOOR_OPEN) < 1e-6,
        )

    # --- Hinge sides -------------------------------------------------------
    ctx.check(
        "upper doors: left hinges left, right hinges right",
        upper_hinges[0].origin.xyz[0] < -0.50
        and upper_hinges[1].origin.xyz[0] > 0.50,
        details=str([h.origin.xyz[0] for h in upper_hinges]),
    )
    ctx.check(
        "lower doors: left hinges left, right hinges right",
        lower_hinges[0].origin.xyz[0] < -0.50
        and lower_hinges[1].origin.xyz[0] > 0.50,
        details=str([h.origin.xyz[0] for h in lower_hinges]),
    )

    # --- Opening pose: doors swing outward ---------------------------------
    with ctx.pose(
        {
            upper_hinges[0]: DOOR_OPEN,
            upper_hinges[1]: DOOR_OPEN,
            lower_hinges[0]: DOOR_OPEN,
            lower_hinges[1]: DOOR_OPEN,
        }
    ):
        open_u0 = ctx.part_world_aabb(upper_doors[0])
        open_l0 = ctx.part_world_aabb(lower_doors[0])
    ctx.check(
        "upper left door swings outward past front face",
        open_u0 is not None and open_u0[1][1] > FRONT_Y + 0.20,
        details=str(open_u0),
    )
    ctx.check(
        "lower left door swings outward past front face",
        open_l0 is not None and open_l0[1][1] > FRONT_Y + 0.20,
        details=str(open_l0),
    )

    # --- Tambour sliding front ---------------------------------------------
    ctx.check(
        "tambour_slide is prismatic",
        tambour_joint.articulation_type == ArticulationType.PRISMATIC,
    )
    tax = tambour_joint.axis
    ctx.check(
        "tambour_slide axis is along X",
        abs(abs(tax[0]) - 1.0) < 1e-9 and abs(tax[1]) < 1e-9 and abs(tax[2]) < 1e-9,
        details=str(tax),
    )
    tlim = tambour_joint.motion_limits
    ctx.check(
        "tambour_slide range 0..~0.76 m",
        tlim is not None and tlim.lower == 0.0 and tlim.upper > 0.50,
        details=str(tlim),
    )

    t_rest = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour at rest sits in the middle zone",
        t_rest is not None
        and t_rest[0][2] > MID_BOT - 0.02
        and t_rest[1][2] < MID_TOP + 0.02,
        details=str(t_rest),
    )

    rest_pos = ctx.part_world_position(tambour)
    with ctx.pose({tambour_joint: TAMBOUR_SLIDE}):
        slid_pos = ctx.part_world_position(tambour)
    ctx.check(
        "tambour slides in +X direction",
        rest_pos is not None
        and slid_pos is not None
        and slid_pos[0] > rest_pos[0] + 0.50,
        details=f"rest={rest_pos}, slid={slid_pos}",
    )

    # --- Caster blocks at base --------------------------------------------
    for i in range(4):
        caabb = ctx.part_element_world_aabb(body, elem=f"caster_{i}")
        ctx.check(
            f"caster_{i} exists near the floor",
            caabb is not None and caabb[0][2] < 0.005,
            details=str(caabb),
        )

    # --- Zone placement ---------------------------------------------------
    for i, door in enumerate(upper_doors):
        daabb = ctx.part_world_aabb(door)
        ctx.check(
            f"upper_door_{i} is in the upper zone",
            daabb is not None
            and daabb[0][2] > UPPER_BOT - 0.05
            and daabb[1][2] < UPPER_TOP + 0.05,
            details=str(daabb),
        )
    for i, door in enumerate(lower_doors):
        # Check leaf element only (not vent backing which may extend slightly)
        laabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"lower_door_{i} leaf is in the lower zone",
            laabb is not None
            and laabb[0][2] > LOWER_BOT - 0.01
            and laabb[1][2] < LOWER_TOP + 0.01,
            details=str(laabb),
        )

    # --- Riveted top cap --------------------------------------------------
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > UPPER_TOP,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
