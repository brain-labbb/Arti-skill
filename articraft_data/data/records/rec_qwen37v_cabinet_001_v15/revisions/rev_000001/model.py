from __future__ import annotations

"""Variant 15: Vintage industrial steel locker cabinet with glass-framed upper
doors and solid lower doors, visible barrel hinges on the door side.

Reference image: picture/Other/Cabinet/001.png

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four short splayed legs and
carries a thin riveted top cap strip. The front is split into four doors, each
with a glass-framed upper section and a solid steel lower section separated by
a horizontal divider rail. The two left doors hinge on their left edges, the two
right doors hinge on their right edges, with visible barrel hinges on the door
side. Each door is an independent revolute joint about a vertical axis, 0..~110
deg outward. Each door carries a small quarter-turn latch knob on the lower
solid section.
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
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
LEG_H = 0.15  # splayed leg height; carcass starts here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03
POCKET_EDGES = [-0.78, -0.4125, -0.3825, -0.015, 0.015, 0.3825, 0.4125, 0.78]

DOOR_W = 0.364  # leaf width
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
HINGE_INSET = 0.0005

# Door split: glass upper / solid lower
SPLIT_RATIO = 0.55  # upper section is 55% of door height
UPPER_H = DOOR_H * SPLIT_RATIO  # ~0.839
LOWER_H = DOOR_H * (1.0 - SPLIT_RATIO)  # ~0.687
LOCAL_BOT = -DOOR_H / 2.0  # door-local z bottom
LOCAL_TOP = DOOR_H / 2.0  # door-local z top
SPLIT_LOCAL_Z = LOCAL_BOT + LOWER_H  # split line in door-local z (~-0.076)
SPLIT_WORLD_Z = DOOR_ZC + SPLIT_LOCAL_Z  # split line in world z (~0.899)

FRAME_BORDER = 0.038  # steel frame border around glass opening
DIVIDER_RAIL_H = 0.028  # horizontal divider rail at split line
GLASS_RABBET = 0.005  # glass extends into frame border (rabbet seat depth)

# Vent slot in the lower solid section
SLOT_LEN = 0.28  # vertical slot length
SLOT_W = 0.028
SLOT_ZC = LOCAL_BOT + LOWER_H * 0.35  # near bottom of lower section

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)

# Latch knob vertical offset (on lower solid section, below split)
LATCH_Z_LOCAL = SPLIT_LOCAL_Z - LOWER_H * 0.20


def _door_frame_solid(sign: float, mesh_name: str):
    """Door leaf as one CadQuery solid: lower solid panel (with rounded-end
    vent slot), upper frame around glass opening, and horizontal divider rail.
    ``sign``=+1 -> panel extends along +X (left-hinged); -1 -> along -X."""
    xc = sign * DOOR_W / 2.0

    # Lower solid panel
    lower_zc = (LOCAL_BOT + SPLIT_LOCAL_Z) / 2.0
    lower = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, LOWER_H)
        .translate((xc, -DOOR_T / 2.0, lower_zc))
    )
    # Rounded-end vent slot cut through lower panel
    cutter = (
        cq.Workplane("XZ")
        .slot2D(SLOT_LEN, SLOT_W, 90)
        .extrude(0.05, both=True)
        .translate((xc, 0.0, SLOT_ZC))
    )
    lower = lower.cut(cutter)

    # Upper frame: solid outer box minus glass opening
    upper_zc = (SPLIT_LOCAL_Z + LOCAL_TOP) / 2.0
    upper_outer = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, UPPER_H)
        .translate((xc, -DOOR_T / 2.0, upper_zc))
    )
    gw = DOOR_W - 2.0 * FRAME_BORDER
    gh = UPPER_H - 2.0 * FRAME_BORDER
    glass_cutter = (
        cq.Workplane("XY")
        .box(gw, DOOR_T + 0.01, gh)
        .translate((xc, -DOOR_T / 2.0, upper_zc))
    )
    upper_frame = upper_outer.cut(glass_cutter)

    # Horizontal divider rail at split line (overlaps both sections)
    divider = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DIVIDER_RAIL_H)
        .translate((xc, -DOOR_T / 2.0, SPLIT_LOCAL_Z))
    )

    leaf = lower.union(upper_frame).union(divider)
    return mesh_from_cadquery(leaf, mesh_name)


def _glass_pane_mesh(mesh_name: str):
    """Thin glass pane seated in the frame rabbet (extends slightly into the
    frame border for a connected mount)."""
    gw = DOOR_W - 2.0 * (FRAME_BORDER - GLASS_RABBET)
    gh = UPPER_H - 2.0 * (FRAME_BORDER - GLASS_RABBET)
    gt = 0.004
    pane = cq.Workplane("XY").box(gw, gt, gh)
    return mesh_from_cadquery(pane, mesh_name)


def _leg_solid(mesh_name: str):
    """Splayed tapered leg: small foot on the floor, wide top embedded into
    the carcass bottom."""
    leg = (
        cq.Workplane("XY")
        .center(0.03, 0.03)
        .rect(0.035, 0.035)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.03, -0.03)
        .rect(0.06, 0.06)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet_v15")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.38, 0.39, 0.41, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))
    glass_mat = model.material("glass_pane", rgba=(0.42, 0.50, 0.56, 0.88))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + legs + front frame + top cap + rivets
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")
    carcass_h = CAB_TOP - LEG_H  # 1.65
    carcass_zc = LEG_H + carcass_h / 2.0

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
        origin=Origin(xyz=(0.0, 0.0, LEG_H + WALL_T / 2.0)),
        material=steel_body,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T / 2.0)),
        material=steel_body,
        name="top_panel",
    )
    # Interior shelf.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T + 0.01, 0.43, 0.015)),
        origin=Origin(xyz=(0.0, -0.02, 0.95)),
        material=steel_body,
        name="interior_shelf",
    )
    # Front frame: bottom rail, top rail, three intermediate stiles.
    body.visual(
        Box((CAB_W - 2.0 * WALL_T, WALL_T, BOTTOM_RAIL_TOP - LEG_H + 0.01)),
        origin=Origin(
            xyz=(0.0, FRONT_Y - WALL_T / 2.0, (LEG_H + BOTTOM_RAIL_TOP) / 2.0)
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
    for i, xc in enumerate((-0.3975, 0.0, 0.3975)):
        body.visual(
            Box((STILE_W, WALL_T, stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
            material=steel_trim,
            name=f"front_stile_{i}",
        )
    # Thin riveted top cap strip with slight overhang.
    body.visual(
        Box((CAB_W + 2 * CAP_OVERHANG, CAB_D + 2 * CAP_OVERHANG, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001 + CAP_T / 2.0)),
        material=steel_trim,
        name="top_cap",
    )
    # Raised rivet dots along the top rail.
    n_riv = 13
    for i in range(n_riv):
        rx = -0.72 + i * (1.44 / (n_riv - 1))
        body.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(rx, FRONT_Y + 0.002, 1.77)),
            material=steel_rivet,
            name=f"rivet_{i}",
        )

    # Splayed legs.
    leg_mesh = _leg_solid("splayed_leg")
    leg_corners = [
        (0.72, 0.19, 0.0),
        (-0.72, 0.19, math.pi / 2.0),
        (-0.72, -0.19, math.pi),
        (0.72, -0.19, 3.0 * math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_corners):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ------------------------------------------------------------------
    # Barrel hinge mesh (reused across all doors, visible on door side)
    # ------------------------------------------------------------------
    barrel_hinge_geom = BarrelHingeGeometry(
        0.065,
        leaf_width_a=0.020,
        leaf_width_b=0.018,
        leaf_thickness=0.0025,
        pin_diameter=0.004,
        knuckle_outer_diameter=0.009,
        knuckle_count=3,
        holes_a=HingeHolePattern(
            style="round", count=2, diameter=0.003, edge_margin=0.008
        ),
        holes_b=HingeHolePattern(
            style="round", count=2, diameter=0.003, edge_margin=0.008
        ),
    )
    barrel_mesh = mesh_from_geometry(barrel_hinge_geom, "barrel_hinge")

    # Glass pane mesh (reused across all doors).
    glass_mesh = _glass_pane_mesh("glass_pane")

    # ------------------------------------------------------------------
    # Four doors: glass-framed upper + solid lower, barrel hinges, latch
    # ------------------------------------------------------------------
    door_specs = [
        (POCKET_EDGES[0] + HINGE_INSET, +1.0, steel_door_a),  # far left
        (POCKET_EDGES[2] + HINGE_INSET, +1.0, steel_door_b),  # centre-left
        (POCKET_EDGES[5] - HINGE_INSET, -1.0, steel_door_b),  # centre-right
        (POCKET_EDGES[7] - HINGE_INSET, -1.0, steel_door_a),  # far right
    ]
    doors = []
    for i, (hinge_x, sign, leaf_mat) in enumerate(door_specs):
        door = model.part(f"door_{i}")
        xc = sign * DOOR_W / 2.0

        # Door frame solid (lower panel + upper frame + divider rail)
        door.visual(
            _door_frame_solid(sign, f"door_frame_{i}"),
            material=leaf_mat,
            name="leaf",
        )

        # Glass pane in upper section (seated in frame rabbet)
        upper_zc = (SPLIT_LOCAL_Z + LOCAL_TOP) / 2.0
        door.visual(
            glass_mesh,
            origin=Origin(xyz=(xc, -DOOR_T / 2.0, upper_zc)),
            material=glass_mat,
            name="glass_pane",
        )

        # Dark backing plate behind the vent slot (recessed dark slot)
        door.visual(
            Box((SLOT_W + 0.014, 0.005, SLOT_LEN + 0.030)),
            origin=Origin(xyz=(xc, -DOOR_T - 0.001, SLOT_ZC)),
            material=steel_dark,
            name="vent_backing",
        )

        # Stamped vent lines near top of lower section (below divider rail)
        for j, dz in enumerate(
            (SPLIT_LOCAL_Z - 0.04, SPLIT_LOCAL_Z - 0.06, SPLIT_LOCAL_Z - 0.08)
        ):
            door.visual(
                Box((0.16, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )

        # Visible barrel hinges on door side (2 per door)
        hinge_z_positions = [
            LOCAL_BOT + 0.12,  # near bottom of door
            LOCAL_TOP - 0.12,  # near top of door
        ]
        for j, hz in enumerate(hinge_z_positions):
            door.visual(
                barrel_mesh,
                origin=Origin(xyz=(0.0, 0.003, hz)),
                material=steel_trim,
                name=f"hinge_barrel_{j}",
            )

        # Door hinge articulation (revolute, vertical axis)
        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Quarter-turn latch knob on lower solid section
        knob = model.part(f"latch_knob_{i}")
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
            f"latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            # On the lower solid section, 0.10 m in from the free edge
            origin=Origin(xyz=(sign * (DOOR_W - 0.10), 0.0, LATCH_Z_LOCAL)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    doors = [object_model.get_part(f"door_{i}") for i in range(4)]
    hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(4)]
    knobs = [object_model.get_part(f"latch_knob_{i}") for i in range(4)]
    latches = [object_model.get_articulation(f"latch_{i}") for i in range(4)]

    # Intentional local overlaps: barrel hinge leaves lap the fixed frame edges
    frame_elems = ["side_wall_0", "front_stile_0", "front_stile_2", "side_wall_1"]
    for i, door in enumerate(doors):
        for j in range(2):  # 2 barrel hinges per door
            ctx.allow_overlap(
                door,
                body,
                elem_a=f"hinge_barrel_{j}",
                elem_b=frame_elems[i],
                reason="Barrel hinge leaf intentionally overlaps the fixed frame edge it pivots on.",
            )

    # --- Overall envelope, true scale, grounded on the floor ----------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "overall width ~1.6 m (cap overhang included)",
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
            1.78 <= z1 <= 1.86,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs rest on the floor", abs(z0) <= 1e-6, details=f"zmin={z0:.5f}")

    # --- Doors: glass upper, solid lower, barrel hinges, articulation -------
    for i, (door, hinge) in enumerate(zip(doors, hinges)):
        # Revolute hinge with vertical axis
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

        # Glass pane exists in the upper section of each door
        gp_aabb = ctx.part_element_world_aabb(door, elem="glass_pane")
        ctx.check(
            f"door_{i} has glass pane in upper section",
            gp_aabb is not None,
            details=str(gp_aabb),
        )
        if gp_aabb is not None:
            # Glass pane is above the split line
            ctx.check(
                f"door_{i} glass pane is above split line",
                gp_aabb[0][2] > SPLIT_WORLD_Z - 0.03,
                details=f"glass_bottom_z={gp_aabb[0][2]:.3f}, split_z={SPLIT_WORLD_Z:.3f}",
            )
            # Glass pane is within the door width
            ctx.check(
                f"door_{i} glass pane narrower than door",
                (gp_aabb[1][0] - gp_aabb[0][0]) < DOOR_W - 0.02,
                details=f"glass_width={gp_aabb[1][0] - gp_aabb[0][0]:.3f}",
            )

        # Visible barrel hinges on door side (2 per door)
        for j in range(2):
            hb_aabb = ctx.part_element_world_aabb(door, elem=f"hinge_barrel_{j}")
            ctx.check(
                f"door_{i} hinge_barrel_{j} visible on door side",
                hb_aabb is not None,
                details=str(hb_aabb),
            )
            if hb_aabb is not None:
                # Barrel hinge is near the door hinge edge (close to hinge origin x)
                hinge_x = hinge.origin.xyz[0]
                barrel_cx = 0.5 * (hb_aabb[0][0] + hb_aabb[1][0])
                ctx.check(
                    f"door_{i} hinge_barrel_{j} near hinge edge",
                    abs(barrel_cx - hinge_x) < 0.04,
                    details=f"barrel_cx={barrel_cx:.3f}, hinge_x={hinge_x:.3f}",
                )

        # Closed leaf sits flush in the front frame plane
        daabb = ctx.part_element_world_aabb(door, elem="leaf")
        ctx.check(
            f"door_{i} closed leaf is flush with the front face",
            daabb is not None
            and abs(daabb[1][1] - FRONT_Y) < 1e-3
            and abs(daabb[0][1] - (FRONT_Y - DOOR_T)) < 1e-3,
            details=str(daabb),
        )
        ctx.expect_within(
            door,
            body,
            axes="x",
            margin=0.012,
            name=f"door_{i} stays inside the cabinet width when closed",
        )

        # Vent backing in the lower solid section
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot in lower solid section",
            vb is not None and vb[1][2] < SPLIT_WORLD_Z and vb[0][2] > DOOR_Z0,
            details=str(vb),
        )

    # Hinge sides: left pair on left edges, right pair on right edges
    ctx.check(
        "left pair hinges on left edges, right pair on right edges",
        hinges[0].origin.xyz[0] < -0.77
        and -0.39 < hinges[1].origin.xyz[0] < -0.37
        and 0.37 < hinges[2].origin.xyz[0] < 0.39
        and hinges[3].origin.xyz[0] > 0.77,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: leaves swing outward, pairs away from centre
    closed0 = ctx.part_world_aabb(doors[0])
    closed3 = ctx.part_world_aabb(doors[3])
    with ctx.pose({hinges[0]: DOOR_OPEN, hinges[3]: DOOR_OPEN}):
        open0 = ctx.part_world_aabb(doors[0])
        open3 = ctx.part_world_aabb(doors[3])
    ctx.check(
        "open leaves swing outward past the front face",
        open0 is not None
        and open3 is not None
        and open0[1][1] > FRONT_Y + 0.25
        and open3[1][1] > FRONT_Y + 0.25,
        details=f"open0={open0}, open3={open3}",
    )
    ctx.check(
        "outer pair opens away from the cabinet centre",
        closed0 is not None
        and closed3 is not None
        and open0[0][0] < closed0[0][0] - 0.05
        and open3[1][0] > closed3[1][0] + 0.05,
        details=f"closed0={closed0}, open0={open0}",
    )

    # --- Latch knobs on lower solid section ---------------------------------
    for i, (knob, latch, door) in enumerate(zip(knobs, latches, doors)):
        ctx.check(
            f"latch_{i} is a quarter-turn revolute about the door normal",
            latch.articulation_type == ArticulationType.REVOLUTE
            and latch.axis == (0.0, 1.0, 0.0)
            and latch.motion_limits is not None
            and abs(latch.motion_limits.upper - math.pi / 2.0) < 1e-6,
        )
        ctx.expect_contact(
            knob,
            door,
            elem_a="backplate",
            elem_b="leaf",
            contact_tol=1e-6,
            name=f"latch_knob_{i} backplate seats on the leaf face",
        )
        # Latch knob is on the lower solid section (below split line)
        knob_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            f"latch_knob_{i} on lower solid section (below split)",
            knob_aabb is not None
            and 0.5 * (knob_aabb[0][2] + knob_aabb[1][2]) < SPLIT_WORLD_Z + 0.02,
            details=f"knob_center_z={0.5*(knob_aabb[0][2]+knob_aabb[1][2]):.3f}",
        )

    # Off-axis handle tip proves the knob really rotates
    tip_rest = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    with ctx.pose({latches[0]: KNOB_TURN}):
        tip_turn = ctx.part_element_world_aabb(knobs[0], elem="handle_tip")
    ctx.check(
        "turning latch_0 sweeps the handle tip sideways and upward",
        tip_rest is not None
        and tip_turn is not None
        and abs(tip_turn[0][0] - tip_rest[0][0]) > 0.012
        and tip_turn[0][2] > tip_rest[0][2] + 0.012,
        details=f"rest={tip_rest}, turned={tip_turn}",
    )

    # Riveted top cap detail present along the top rail
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
