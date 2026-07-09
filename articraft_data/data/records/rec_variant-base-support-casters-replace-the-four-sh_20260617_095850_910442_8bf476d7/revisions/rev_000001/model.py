from __future__ import annotations

"""Vintage industrial steel locker cabinet with four full-height hinged doors.
Caster variant: four swivel caster wheels replace the original splayed legs.

Reference image: picture/Other/Cabinet/001.png

Overall envelope ~1.6 m wide x 0.5 m deep x ~1.8 m tall, brushed/tarnished raw
steel. A hollow thin-wall (~0.02 m) carcass sits on four swivel caster wheels
(70 mm diameter, continuous-rotation axle) and carries a thin riveted top cap
strip. The front is split into four flat doors separated by narrow stiles. The
two left doors hinge on their left edges, the two right doors hinge on their
right edges (piano-hinge knuckle columns on the outer door edges), so the pairs
swing open away from the cabinet centre. Each door is an independent revolute
joint about a vertical axis, 0..~110 deg outward, and each door carries a small
quarter-turn latch knob at mid-height, a dark recessed ventilation slot with
rounded ends near the bottom, and a group of stamped vent lines near the top.
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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet is centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 1.60  # overall carcass width  (X)
CAB_D = 0.50  # overall carcass depth  (Y)
CAB_TOP = 1.80  # carcass top height   (Z)
LEG_H = 0.15  # caster height; carcass bottom sits here
WALL_T = 0.02  # thin steel wall thickness

FRONT_Y = CAB_D / 2.0  # +0.25, front face plane
BACK_Y = -CAB_D / 2.0

BOTTOM_RAIL_TOP = LEG_H + 0.06  # 0.21
TOP_RAIL_BOT = CAB_TOP - 0.06  # 1.74

STILE_W = 0.03
# Door pockets between the inner faces of the side walls (x = -0.78 .. 0.78)
POCKET_EDGES = [-0.78, -0.4125, -0.3825, -0.015, 0.015, 0.3825, 0.4125, 0.78]

DOOR_W = 0.364  # leaf width (pocket is 0.3675 wide -> swing clearance)
DOOR_T = WALL_T
DOOR_Z0 = BOTTOM_RAIL_TOP + 0.002  # 0.212
DOOR_Z1 = TOP_RAIL_BOT - 0.002  # 1.738
DOOR_H = DOOR_Z1 - DOOR_Z0  # 1.526
DOOR_ZC = 0.5 * (DOOR_Z0 + DOOR_Z1)  # 0.975
HINGE_INSET = 0.0005  # hinge line inset from the pocket edge

SLOT_LEN = 0.36  # dark rounded-end vent slot near the bottom
SLOT_W = 0.030
SLOT_ZC = -0.40  # in door-local z (door centre = 0)

# Caster dimensions (replacing splayed legs).
WHEEL_R = 0.035  # caster wheel radius (70 mm dia)
WHEEL_W = 0.030  # caster wheel width
FORK_GAP = 0.036  # gap between fork arms (clearance for wheel + hub bearing races)
FORK_ARM_T = 0.006  # fork arm thickness
CASTER_AXLE_Z = WHEEL_R  # world z of axle centre (wheel bottom at floor)
# Local z of axle relative to fork mounting face (top of plate = local 0).
FORK_AXLE_LOCAL_Z = CASTER_AXLE_Z - LEG_H  # -0.115

BARREL_R = 0.0075
KNUCKLE_R = 0.0095
BARREL_LEN = DOOR_H - 0.03

CAP_T = 0.022  # riveted top cap strip
CAP_OVERHANG = 0.02

DOOR_OPEN = math.radians(110.0)
KNOB_TURN = math.radians(90.0)


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
    """Piano-hinge knuckle column along the door hinge edge (local Z axis)."""
    barrel = cq.Workplane("XY").circle(BARREL_R).extrude(BARREL_LEN / 2.0, both=True)
    ring_h = 0.055
    for zc in (-0.60, -0.30, 0.0, 0.30, 0.60):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _caster_fork_solid(mesh_name: str):
    """Swivel-caster fork assembly: square mounting plate at local z=0 (flush
    against the carcass bottom), cylindrical swivel raceway, crossbar, and two
    flat fork arms extending down past the axle.  All geometry extends in -Z
    from the mounting face."""
    plate_sz = 0.060
    plate_t = 0.006
    # Mounting plate (top face at z=0).
    plate = (
        cq.Workplane("XY")
        .box(plate_sz, plate_sz, plate_t)
        .translate((0.0, 0.0, -plate_t / 2.0))
    )
    # Swivel raceway / housing cylinder below plate.
    housing_r = 0.015
    housing_h = 0.018
    housing = (
        cq.Workplane("XY")
        .circle(housing_r)
        .extrude(housing_h)
        .translate((0.0, 0.0, -plate_t - housing_h))
    )
    # Crossbar connecting housing to fork arms.
    crossbar_w = FORK_GAP + 2.0 * FORK_ARM_T
    crossbar_d = 0.034
    crossbar_t = 0.008
    crossbar_z_top = -plate_t - housing_h
    crossbar = (
        cq.Workplane("XY")
        .box(crossbar_w, crossbar_d, crossbar_t)
        .translate((0.0, 0.0, crossbar_z_top - crossbar_t / 2.0))
    )
    # Two fork arms extending down past the axle location.
    arm_drop = abs(FORK_AXLE_LOCAL_Z) + 0.016 - (plate_t + housing_h + crossbar_t)
    arm_bottom_z = crossbar_z_top - crossbar_t - arm_drop
    for sign in (-1.0, 1.0):
        ax = sign * (FORK_GAP / 2.0 + FORK_ARM_T / 2.0)
        arm = (
            cq.Workplane("XY")
            .box(FORK_ARM_T, crossbar_d, arm_drop)
            .translate((ax, 0.0, crossbar_z_top - crossbar_t - arm_drop / 2.0))
        )
        crossbar = crossbar.union(arm)
    result = plate.union(housing).union(crossbar)
    return mesh_from_cadquery(result, mesh_name)


def _caster_wheel_mesh(mesh_name: str):
    """Small industrial caster wheel using the WheelGeometry helper.
    The hub is wider than the wheel rim and extends slightly past the fork
    arm inner faces so the bearing races provide the physical support
    (small intentional press-fit overlap with the fork arms)."""
    hub_w = FORK_GAP + 0.004  # 2 mm press-fit per side
    wheel = WheelGeometry(
        WHEEL_R,
        WHEEL_W,
        hub=WheelHub(radius=0.014, width=hub_w, cap_style="flat"),
        face=WheelFace(dish_depth=0.003, front_inset=0.001, rear_inset=0.001),
        bore=WheelBore(style="round", diameter=0.008),
    )
    return mesh_from_geometry(wheel, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_steel_locker_cabinet")

    steel_body = model.material("steel_body", rgba=(0.60, 0.61, 0.63, 1.0))
    steel_door_a = model.material("steel_door_a", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.50, 0.51, 0.54, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.46, 0.47, 0.49, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.18, 0.18, 0.20, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.52, 0.53, 0.55, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass + casters + front frame + top cap + rivets
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
    # Back wall (overlaps side walls slightly so the shell is one body).
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
    # Interior shelf (thin, embedded into side and back walls).
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
    stile_h = TOP_RAIL_BOT - BOTTOM_RAIL_TOP + 0.01  # overlaps both rails
    for i, xc in enumerate((-0.3975, 0.0, 0.3975)):
        body.visual(
            Box((STILE_W, WALL_T, stile_h)),
            origin=Origin(xyz=(xc, FRONT_Y - WALL_T / 2.0, DOOR_ZC)),
            material=steel_trim,
            name=f"front_stile_{i}",
        )
    # Thin riveted top cap strip with a slight overhang.
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

    # Swivel caster assemblies: fork + mounting plate as body visuals,
    # wheel as a separate part with a continuous revolute joint on its axle.
    caster_mat = model.material("caster_steel", rgba=(0.32, 0.33, 0.36, 1.0))
    caster_wheel_mat = model.material("caster_wheel", rgba=(0.14, 0.14, 0.16, 1.0))
    fork_mesh = _caster_fork_solid("caster_fork")
    wheel_mesh = _caster_wheel_mesh("caster_wheel")
    caster_corners = [
        (0.72, 0.19),   # front-right
        (-0.72, 0.19),  # front-left
        (-0.72, -0.19), # rear-left
        (0.72, -0.19),  # rear-right
    ]
    for i, (cx, cy) in enumerate(caster_corners):
        # Fork + plate as static body visual (mounted flush under carcass).
        body.visual(
            fork_mesh,
            origin=Origin(xyz=(cx, cy, LEG_H)),
            material=caster_mat,
            name=f"caster_fork_{i}",
        )
        # Wheel part with continuous revolute about world X (axle direction).
        wheel_part = model.part(f"caster_wheel_{i}")
        wheel_part.visual(
            wheel_mesh,
            material=caster_wheel_mat,
            name="wheel",
        )
        model.articulation(
            f"caster_{i}_axle",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_part,
            origin=Origin(xyz=(cx, cy, CASTER_AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=10.0),
        )

    # ------------------------------------------------------------------
    # Four doors. sign=+1: hinge on the door's left edge, panel extends +X.
    # sign=-1: hinge on the door's right edge, panel extends -X.
    # ------------------------------------------------------------------
    door_specs = [
        # (hinge world x, sign, leaf material)
        (POCKET_EDGES[0] + HINGE_INSET, +1.0, steel_door_a),  # far left
        (POCKET_EDGES[2] + HINGE_INSET, +1.0, steel_door_b),  # centre-left
        (POCKET_EDGES[5] - HINGE_INSET, -1.0, steel_door_b),  # centre-right
        (POCKET_EDGES[7] - HINGE_INSET, -1.0, steel_door_a),  # far right
    ]
    doors = []
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
        for j, dz in enumerate((0.60, 0.62, 0.64)):
            door.visual(
                Box((0.16, 0.004, 0.006)),
                origin=Origin(xyz=(xc, -0.0012, dz)),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Piano-hinge knuckle column on the hinge edge (moves with the door;
        # its outer half intentionally laps the fixed frame edge).
        door.visual(
            _hinge_barrel_solid(f"hinge_barrel_{i}"),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, FRONT_Y, DOOR_ZC)),
            # +Z opens a left-hinged leaf outward; mirrored leaves use -Z so
            # positive q always swings the free edge out of the cabinet.
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(
                effort=40.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
            ),
        )
        doors.append(door)

        # Quarter-turn latch knob at mid-height near the free edge.
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
        # Off-axis teardrop tip at the lower end of the bar.
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
            # On the door front face, 0.10 m in from the free edge.
            origin=Origin(xyz=(sign * (DOOR_W - 0.10), 0.0, 0.0)),
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
    wheels = [object_model.get_part(f"caster_wheel_{i}") for i in range(4)]
    axles = [object_model.get_articulation(f"caster_{i}_axle") for i in range(4)]

    # Intentional local laps: each door's hinge knuckle column embeds a few
    # tenths of a millimetre into the fixed frame edge it hinges on (captured
    # piano-hinge knuckles), like a real continuous hinge.
    frame_elems = ["side_wall_0", "front_stile_0", "front_stile_2", "side_wall_1"]
    for door, elem in zip(doors, frame_elems):
        ctx.allow_overlap(
            door,
            body,
            elem_a="hinge_barrel",
            elem_b=elem,
            reason="Piano-hinge knuckle column intentionally laps the fixed frame edge it pivots on.",
        )

    # --- Overall envelope, true scale, grounded on caster wheels -------------
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

    # --- Caster wheels: existence, continuous joints, floor contact ----------
    # Bearing-race press-fit: the wheel hub extends slightly past the fork arm
    # inner faces (small intentional overlap representing bearing interference).
    for i, wheel in enumerate(wheels):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=f"caster_fork_{i}",
            elem_b="wheel",
            reason="Wheel hub bearing races press-fit against the fork arm inner faces.",
        )

    for i, (wheel, axle) in enumerate(zip(wheels, axles)):
        ctx.check(
            f"caster_{i}_axle is continuous revolute",
            axle.articulation_type == ArticulationType.CONTINUOUS,
        )
        ax = axle.axis
        ctx.check(
            f"caster_{i} axle is horizontal (world X)",
            abs(ax[0] - 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
            details=str(ax),
        )
        # Wheel bottom should be at or very near the floor.
        waabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"caster_wheel_{i} touches the floor",
            waabb is not None and waabb[0][2] <= 0.002,
            details=str(waabb),
        )
        # Wheel centre is at caster axle height.
        ctx.check(
            f"caster_{i} axle origin at wheel centre height",
            abs(axle.origin.xyz[2] - CASTER_AXLE_Z) < 1e-6,
            details=f"axle_z={axle.origin.xyz[2]:.4f}",
        )
        # Prove the wheel hub is captured between the fork arms in X.
        ctx.expect_within(
            wheel,
            body,
            axes="x",
            margin=0.005,
            name=f"caster_wheel_{i} hub stays within the fork arms",
        )
    # Caster fork visuals are present on the body (4 fork elements).
    for i in range(4):
        fork_vis = body.get_visual(f"caster_fork_{i}")
        ctx.check(
            f"caster_fork_{i} visual exists on cabinet body",
            fork_vis is not None,
        )

    # --- Doors: geometry, hinge type/axis/range, closed seating -------------
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
        # Closed leaf sits flush in the front frame plane, not sunk or proud.
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
        # Recessed dark vent slot near the bottom of the leaf.
        vb = ctx.part_element_world_aabb(door, elem="vent_backing")
        ctx.check(
            f"door_{i} vent slot sits in the lower half of the leaf",
            vb is not None and vb[1][2] < DOOR_ZC and vb[0][2] > DOOR_Z0,
            details=str(vb),
        )

    # Hinge sides: outer doors hinge at the cabinet's outer edges, the centre
    # pair hinges beside the outer doors, so all free edges face the centre.
    ctx.check(
        "left pair hinges on left edges, right pair on right edges",
        hinges[0].origin.xyz[0] < -0.77
        and -0.39 < hinges[1].origin.xyz[0] < -0.37
        and 0.37 < hinges[2].origin.xyz[0] < 0.39
        and hinges[3].origin.xyz[0] > 0.77,
        details=str([h.origin.xyz[0] for h in hinges]),
    )

    # Opening pose: every leaf swings outward (+Y), pairs swing away from the
    # cabinet centre (leftmost leaf ends further left, rightmost further right).
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

    # --- Latch knobs ---------------------------------------------------------
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
        ctx.check(
            f"latch_knob_{i} sits at door mid-height",
            (
                lambda a: a is not None and abs(0.5 * (a[0][2] + a[1][2]) - DOOR_ZC) < 0.03
            )(ctx.part_world_aabb(knob)),
        )

    # Off-axis handle tip proves the knob really rotates about the door normal.
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
