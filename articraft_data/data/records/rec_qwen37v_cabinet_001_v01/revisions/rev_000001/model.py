from __future__ import annotations

"""Tall two-door industrial steel cabinet with raised plinth base.

Variant of the vintage steel locker: reconfigured as a tall two-door cabinet
(~1.0 m wide × 0.5 m deep × ~1.9 m tall), brushed/tarnished raw steel. A hollow
thin-wall (~0.02 m) carcass sits on a solid raised plinth base (~0.12 m tall)
and carries a thin riveted top cap strip. The front is split into two full-height
flat doors separated by a narrow centre stile. The left door hinges on its left
edge, the right door hinges on its right edge, with visible barrel hinges on the
door side, so the pair swings open away from the cabinet centre. Each door is an
independent revolute joint about a vertical axis, 0..~110 deg outward. Each door
carries a separate D-shaped drawer pull handle at mid-height near the free edge,
a dark recessed ventilation slot with rounded ends near the bottom, and a group
of stamped vent lines near the top.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    HingeHolePattern,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.00  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.90  # carcass top height   (Z)
PLINTH_H = 0.12  # plinth base height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = PLINTH_H + 0.06  # 0.18
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.84

STILE_W = 0.03

# Two-door layout: inner pocket from side wall inner face to centre stile edge.
SIDE_INNER_X = CAB_W / 2.0 - WALL_T  # 0.48
DOOR_W = SIDE_INNER_X - STILE_W / 2.0 - 0.003  # ~0.462 per leaf
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.182
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.838
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.656
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 1.010

SLOT_LEN = 0.40  # dark rounded-end vent slot near the bottom
SLOT_W = 0.030
SLOT_ZC = -0.50  # in door-local z (door centre = 0)

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)

# Plinth dimensions: slightly inset from the carcass for a toe-kick look.
PLINTH_INSET = 0.03
PLINTH_W = CAB_W - 2.0 * PLINTH_INSET
PLINTH_D = CAB_D - PLINTH_INSET


def _door_solid(sign: float, mesh_name: str):
    """Door leaf as one CadQuery solid: flat panel with a rounded-end
    through slot near the bottom. ``sign``=+1 -> panel extends along +X from
    the hinge line (left-hinged); ``sign``=-1 -> along -X (right-hinged)."""
    xc = sign * DOOR_W / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((xc, -DOOR_T / 2.0, 0.0))
    )
    # Vertical slot, rounded ends, cut through the leaf thickness.
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    leaf = panel.cut(cutter)
    return mesh_from_cadquery(leaf, mesh_name)


def _hinge_barrel_solid(mesh_name: str):
    """Visible hinge barrel: a cylindrical pin column with knuckle rings,
    built along local Z axis. More prominent than piano-hinge knuckles."""
    barrel_r = 0.010
    knuckle_r = 0.014
    barrel_len = DOOR_H - 0.04
    barrel = (
        cq.Workplane("XY")
        .circle(barrel_r)
        .extrude(barrel_len / 2.0, both=True)
    )
    # Knuckle rings every ~0.25 m along the barrel.
    ring_h = 0.040
    n_rings = max(3, int(barrel_len / 0.25))
    spacing = barrel_len / (n_rings + 1)
    for i in range(1, n_rings + 1):
        zc = -barrel_len / 2.0 + i * spacing
        ring = (
            cq.Workplane("XY")
            .circle(knuckle_r)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _pull_handle_solid(mesh_name: str):
    """D-shaped drawer pull handle: a horizontal bar with two mounting posts.
    Built in local frame: mounting face at z=0, everything extends along +Z
    (which will be rotated to point outward from the door front face)."""
    bar_w = 0.10  # handle width
    bar_h = 0.012  # bar cross-section height
    bar_d = 0.014  # bar cross-section depth
    post_r = 0.005
    post_len = 0.022
    post_spacing = 0.07
    plate_t = 0.003

    # Mounting plates (rosettes) sit at z=0..plate_t.
    handle = cq.Workplane("XY")
    for sx in (-1.0, 1.0):
        plate = (
            cq.Workplane("XY")
            .circle(0.010)
            .extrude(plate_t)
            .translate((sx * post_spacing / 2.0, 0.0, 0.0))
        )
        if handle.val() is None or not hasattr(handle.val(), "Volume"):
            handle = plate
        else:
            handle = handle.union(plate)

    # Posts extend from plate top upward.
    for sx in (-1.0, 1.0):
        post = (
            cq.Workplane("XY")
            .circle(post_r)
            .extrude(post_len)
            .translate((sx * post_spacing / 2.0, 0.0, plate_t))
        )
        handle = handle.union(post)

    # Horizontal bar on top of posts.
    bar = (
        cq.Workplane("XY")
        .box(bar_w, bar_d, bar_h)
        .translate((0.0, 0.0, plate_t + post_len + bar_h / 2.0))
    )
    bar = bar.edges("|X").fillet(0.003)
    handle = handle.union(bar)

    return mesh_from_cadquery(handle, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_two_door_steel_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_plinth = model.material("steel_plinth", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_handle = model.material("steel_handle", rgba=(0.22, 0.22, 0.24, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    steel_hinge = model.material("steel_hinge", rgba=(0.42, 0.43, 0.45, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: plinth base + hollow carcass + front frame + top cap
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - PLINTH_H  # 1.78
    carcass_zc = PLINTH_H + carcass_h / 2.0

    # Raised plinth base: solid box, slightly inset for toe-kick appearance.
    body.visual(
        Box((PLINTH_W, PLINTH_D, PLINTH_H)),
        origin=Origin(xyz=(0.0, -PLINTH_INSET / 2.0, PLINTH_H / 2.0)),
        material=steel_plinth,
        name="plinth_base",
    )
    # Plinth top lip (thin trim strip around the top edge of the plinth).
    body.visual(
        Box((CAB_W + 0.01, CAB_D + 0.01, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + 0.004)),
        material=steel_trim,
        name="plinth_lip",
    )

    # Side walls (full depth, full carcass height).
    for sx, vname in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((WALL_T, CAB_D, carcass_h)),
            origin=Origin(xyz=(sx * (CAB_W / 2.0 - WALL_T / 2.0), 0.0, carcass_zc)),
            material=steel_body,
            name=vname,
        )
    # Back wall.
    body.visual(
        Box((CAB_W - WALL_T, WALL_T, carcass_h - 0.02)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, carcass_zc)),
        material=steel_body,
        name="back_wall",
    )
    # Bottom and top panels.
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Interior shelf (thin, embedded into side and back walls).
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.43, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, 1.00)),
        material=steel_body,
        name="interior_shelf",
    )
    # Front frame: bottom rail, top rail, one centre stile.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - PLINTH_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (PLINTH_H + BOTTOM_RAIL_TOP) / 2.0)
        ),
        material=steel_body,
        name="front_bottom_rail",
    )
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, CAB_TOP - TOP_RAIL_BOT + 0.01)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, (TOP_RAIL_BOT + CAB_TOP) / 2.0)),
        material=steel_body,
        name="front_top_rail",
    )
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01
    body.visual(
        Box((STILE_W, WALL_T, stile_h)),
        origin=Origin(xyz=(0.0, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
        material=steel_trim,
        name="centre_stile",
    )

    # Thin riveted top cap strip with a slight overhang.
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the top rail.
    n_riv = 9
    for i in range(n_riv):
        rx = -0.42 + i * (0.84 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.87)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # ------------------------------------------------------------------
    # Two doors. sign=+1: hinge on left edge, panel extends +X.
    # sign=-1: hinge on right edge, panel extends -X.
    # ------------------------------------------------------------------
    hinge_x_left = -(CAB_W / 2.0 - WALL_T) + 0.001  # left pocket inner edge
    hinge_x_right = (CAB_W / 2.0 - WALL_T) - 0.001  # right pocket inner edge

    door_specs = [
        # (hinge world x, sign, leaf material)
        (hinge_x_left, +1.0, steel_door_a),   # left door, hinges on left
        (hinge_x_right, -1.0, steel_door_b),  # right door, hinges on right
    ]

    handle_mesh = _pull_handle_solid("pull_handle")
    hinge_barrel_mesh = _hinge_barrel_solid("hinge_barrel")

    doors = []
    handles = []
    for i, (hinge_x, sign, leaf_mat) in enumerate(door_specs):
        door = model.part(f"door_{i}")
        xc = sign * DOOR_W / 2.0  # door-local panel centre

        door.visual(
            _door_solid(sign, f"door_leaf_{i}"),
            material=leaf_mat,
            name="leaf",
        )
        # Dark backing plate behind the through slot -> recessed dark slot.
        door.visual(
            Box((SLOT_W + 0.016, 0.005, SLOT_LEN + 0.036)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )
        # Stamped vent lines near the top (slightly proud thin dark strips).
        for j, dz in enumerate((0.68, 0.70, 0.72)):
            door.visual(
                Box((0.20, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Visible hinge barrel on the door side (moves with the door).
        door.visual(
            hinge_barrel_mesh,
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
            material=steel_hinge,
            name="hinge_barrel",
        )

        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            # +Z opens a left-hinged leaf outward; -Z for right-hinged.
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Separate drawer pull handle at mid-height near the free edge.
        # Handle is built with +Z as outward direction; rotate so +Z maps to
        # door-local +Y (outward from front face): rpy=(-pi/2, 0, 0).
        handle = model.part(f"pull_handle_{i}")
        handle.visual(
            handle_mesh,
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel_handle,
            name="handle_body",
        )
        # Mount the handle on the door front face, near the free edge at mid-height.
        handle_x_local = sign * (DOOR_W - 0.08)
        model.articulation(
            f"handle_{i}_mount",
            ArticulationType.FIXED,
            parent=door,
            child=handle,
            origin=Origin(xyz=(handle_x_local, 0.0, 0.0)),
        )
        handles.append(handle)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(2)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(2)]
    handles = [object_model.get_part(f"pull_handle_{i}") for i in range(2)]

    # Intentional local laps: each door's hinge barrel embeds slightly into
    # the fixed frame edge it pivots on (captured barrel hinge knuckles).
    frame_elems = ["side_wall_0", "side_wall_1"]
    for door, elem in zip(doors, frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Visible hinge barrel intentionally laps the fixed frame edge it pivots on.",
        )

    # --- Overall envelope, true scale, grounded on the floor ----------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.0 m",
            0.96 <= (x1 - x0) <= 1.10,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "overall depth ~0.5 m",
            0.46 <= (y1 - y0) <= 0.58,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "overall height ~1.9 m",
            1.86 <= z1 <= 1.98,
            details=f"top={z1:.3f}",
        )
        ctx.check("plinth rests on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Plinth base exists and is at the bottom ---
    plinth_aabb = ctx.part_element_world_aabb(body, elem="plinth_base")
    ctx.check(
        "plinth base present and at the bottom",
        plinth_aabb is not None and plinth_aabb[0][2] < 0.001,
        details=str(plinth_aabb),
    )

    # --- Doors: exactly two, geometry, hinge type/axis/range, closed seating ---
    ctx.check(
        "exactly two doors",
        len(doors) == 2,
    )
    for i, (door, hinge) in enumerate(zip(doors, hinges)):
        ctx.check(
            f"door_{i} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"door_{i} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"door_{i} opens 0..~110 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(110.0)) < 1e-6,
        )
        # Closed leaf sits flush in the front frame plane.
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"door_{i} closed leaf is flush with the front face",
            daabb is not None
            and abs(daabb[1][1] - FRONT_Y) < 1e-4
            and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-4,
            details=str(daabb),
        )
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.012,
            name=f"door_{i} stays inside the cabinet width when closed",
        )

    # Hinge sides: left door hinges at the left edge, right door at the right edge.
    ctx.check(
        "left door hinges on left edge, right door on right edge",
        hinges[0].origin.xyz[0] < -0.40
        and hinges[1].origin.xyz[0] > 0.40,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Visible hinge barrels present on each door.
    for i, door in enumerate(doors):
        hb = ctx.part_element_world_aabb(door, elem="hinge_barrel")
        ctx.check(
            f"door_{i} has visible hinge barrel",
            hb is not None and (hb[1][2] - hb[0][2]) > DOOR_H * 0.8,
            details=str(hb),
        )

    # Opening pose: both leaves swing outward (+Y), away from centre.
    closed0 = ctx.part_world_aabb(doors[0])
    closed1 = ctx.part_world_aabb(doors[1])
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[1]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open1 = ctx.part_world_aabb(doors[1])
    ctx.check(
        "open leaves swing outward past the front face",
        open0 is not None
        and open1 is not None
        and open0[1][1] > FRONT_Y + 0.20
        and open1[1][1] > FRONT_Y + 0.20,
        details=f"open0={open0}, open1={open1}",
    )
    ctx.check(
        "pair opens away from the cabinet centre",
        closed0 is not None
        and closed1 is not None
        and open0[0][0] < closed0[0][0] - 0.05
        and open1[1][0] > closed1[1][0] + 0.05,
        details=f"closed0={closed0}, open0={open0}",
    )

    # --- Drawer pull handles ---
    for i, handle in enumerate(handles):
        haabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"pull_handle_{i} exists and has geometry",
            haabb is not None,
            details=str(haabb),
        )
        # Handle should be near the door mid-height.
        ctx.check(
            f"pull_handle_{i} sits at door mid-height",
            haabb is not None
            and abs(0.5 * (haabb[0][2] + haabb[1][2]) - DOOR_ZC) < 0.06,
            details=str(haabb),
        )
        # Handle protrudes outward from the door front face.
        ctx.expect_gap(
            handle,
            doors[i],
            axis="y",
            min_gap=-0.004,
            max_gap=0.05,
            positive_elem="handle_body",
            negative_elem="leaf",
            name=f"pull_handle_{i} protrudes outward from the door face",
        )
        # Handle bar should extend beyond the leaf front face.
        leaf_aabb = ctx.part_element_world_aabb(doors[i], elem="leaf")
        ctx.check(
            f"pull_handle_{i} bar stands proud of the leaf",
            haabb is not None
            and leaf_aabb is not None
            and haabb[1][1] > leaf_aabb[1][1] + 0.015,
            details=f"handle_y_max={haabb[1][1] if haabb else None}, leaf_y_max={leaf_aabb[1][1] if leaf_aabb else None}",
        )

    # Recessed dark vent slot near the bottom of each leaf.
    for i, door in enumerate(doors):
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot sits in the lower half of the leaf",
            vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
            details=str(vb),
        )

    # Riveted top cap detail present along the top rail.
    rivet_aabb = ctx.part_element_world_aabb(body, elem="rivet_0")
    ctx.check(
        "rivet dots stand proud of the top rail face",
        rivet_aabb is not None
        and rivet_aabb[1][1] > FRONT_Y + 0.003
        and rivet_aabb[0][2] > TOP_RAIL_BOT,
        details=str(rivet_aabb),
    )

    return ctx.report()


object_model = build_object_model()
