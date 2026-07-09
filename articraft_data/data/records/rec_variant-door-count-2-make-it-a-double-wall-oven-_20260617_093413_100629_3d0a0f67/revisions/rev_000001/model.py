from __future__ import annotations

"""Built-in double electric wall oven — two oven cavities stacked vertically.

Articraft brief:
- Object: built-in double electric wall oven, fascia 0.60 m wide x 1.20 m tall,
  body 0.55 m deep overall (hollow box recessed behind the front fascia).
- Root/support: oven_body (hollow shell + front fascia + control strip),
  grounded at z=0; +Y points into the cabinetry, -Y is the user-facing front.
- Parts: oven_body (root), door_0 and door_1 (drop-down, revolute on each
  bottom edge), shelf_rack_0 and shelf_rack_1 (wire racks on prismatic slides).
- Articulations (uniform hinge policy for all ovens):
  * body_to_door_{i}: REVOLUTE, hinge along the door bottom front edge,
    axis +X so positive q swings the top outward (-Y) and down,
    limits 0 (closed vertical) .. pi/2 (open horizontal).
  * body_to_shelf_rack_{i}: PRISMATIC, axis (0,-1,0), travel 0.35 m.
- Visible geometry: matte light-gray steel fascia and shell, dark glass touch
  strip with centered clock display and flanking touch icons (top area),
  two gray door frames each with large frosted-white glass window, brushed-
  aluminum bar handles on two posts near each door top, dark hollow cavities
  with side rails.
- Intentional overlaps: door hinge arms embed into body slots (scoped
  allowances); layered control-strip visuals embed within one part.
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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
FASCIA_W = 0.60
FASCIA_H = 1.20
FASCIA_T = 0.022  # y: -0.020 .. +0.002 (embeds 2 mm into the shell front)
BODY_W = 0.56
BODY_H = 1.16  # z: 0.02 .. 1.18
BODY_D = 0.53  # y: 0.00 .. 0.53
TOTAL_D = 0.55  # fascia front (-0.02) to shell back (0.53)

N_OVENS = 2
UNIT_PITCH = 0.492  # vertical spacing between successive hinge lines

OPEN_W = 0.565  # fascia door opening width
OPEN_H = 0.432  # fascia door opening height

DOOR_W = 0.555
DOOR_T = 0.040
DOOR_H = 0.425  # local z 0.003 .. 0.428

HINGE_Y = -0.045  # door front face plane (hinge y in world)

RACK_TRAVEL = 0.35

# Per-oven hinge z values (bottom edge of each door opening)
OVEN_HINGE_Z = [0.058 + i * UNIT_PITCH for i in range(N_OVENS)]  # [0.058, 0.550]

# Control strip centered near the top of the fascia
CTRL_Z_CENTER = FASCIA_H - 0.0575  # ~1.1425


def _cyl(radius: float, length: float, axis: str, center: tuple[float, float, float]):
    wp = cq.Workplane("XY").cylinder(length, radius)  # long axis Z
    if axis == "x":
        wp = wp.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        wp = wp.rotate((0, 0, 0), (1, 0, 0), 90)
    return wp.translate(center)


# ------------------------------------------------------ shared body geometry

def _body_shell() -> cq.Workplane:
    """Hollow outer shell with one cavity cutout per oven unit."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0, BODY_D / 2.0, 0.02 + BODY_H / 2.0))
    )
    shell = outer
    for hz in OVEN_HINGE_Z:
        cz = hz + 0.20  # cavity center is 0.20 m above the hinge line
        cavity = (
            cq.Workplane("XY")
            .box(0.50, 0.51, 0.40)
            .translate((0, 0.245, cz))
        )
        shell = shell.cut(cavity)
    return shell


def _front_fascia() -> cq.Workplane:
    """Front fascia plate with one rectangular opening per oven unit."""
    plate = (
        cq.Workplane("XY")
        .box(FASCIA_W, FASCIA_T, FASCIA_H)
        .translate((0, -FASCIA_T / 2.0 + 0.002, FASCIA_H / 2.0))
    )
    fascia = plate
    for hz in OVEN_HINGE_Z:
        oz = hz + OPEN_H / 2.0
        opening = (
            cq.Workplane("XY")
            .box(OPEN_W, 0.08, OPEN_H)
            .translate((0, -FASCIA_T / 2.0 + 0.002, oz))
        )
        fascia = fascia.cut(opening)
    return fascia


def _cavity_liner(hinge_z: float) -> cq.Workplane:
    """Thin liner shell for one oven cavity at the given hinge z."""
    cz = hinge_z + 0.20
    outer = (
        cq.Workplane("XY")
        .box(0.504, 0.502, 0.404)
        .translate((0, 0.247, cz))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.470, 0.530, 0.370)
        .translate((0, 0.215, cz))
    )
    return outer.cut(inner)


# ------------------------------------------------------- shared door geometry

def _door_frame() -> cq.Workplane:
    """Door frame panel with a large inset window cutout (local door frame)."""
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0, DOOR_T / 2.0, 0.003 + DOOR_H / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(0.42, 0.08, 0.30)
        .translate((0, DOOR_T / 2.0, 0.185))
    )
    return panel.cut(window)


# ------------------------------------------------------- shared rack geometry

def _wire_rack() -> cq.Workplane:
    """Chrome wire rack in its own local frame (flat, centered at origin)."""
    rack = _cyl(0.0045, 0.40, "y", (0.230, 0.20, 0.0))
    rack = rack.union(_cyl(0.0045, 0.40, "y", (-0.230, 0.20, 0.0)))
    for y in (0.006, 0.20, 0.394):
        rack = rack.union(_cyl(0.004, 0.464, "x", (0.0, y, 0.0)))
    for i in range(9):
        x = -0.184 + 0.046 * i
        rack = rack.union(_cyl(0.003, 0.388, "y", (x, 0.20, 0.0)))
    return rack


# ---------------------------------------------------------- part constructors

def _add_door(model, body, index: int, hinge_z: float):
    """Create one door part + revolute articulation for oven unit *index*."""
    door = model.part(f"door_{index}")
    door.visual(
        mesh_from_cadquery(_door_frame(), f"door_frame_{index}"),
        material="door_gray",
        name="door_frame",
    )
    door.visual(
        Box((0.44, 0.008, 0.32)),
        origin=Origin(xyz=(0.0, DOOR_T / 2.0, 0.185)),
        material="frosted_glass",
        name="window_glass",
    )
    # brushed-aluminum bar handle on two short posts near the top edge
    door.visual(
        Cylinder(radius=0.0125, length=0.45),
        origin=Origin(xyz=(0.0, -0.040, 0.392), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="aluminum",
        name="handle_bar",
    )
    for j, sx in enumerate((-1.0, 1.0)):
        door.visual(
            Cylinder(radius=0.009, length=0.036),
            origin=Origin(xyz=(sx * 0.185, -0.014, 0.392), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="aluminum",
            name=f"handle_post_{j}",
        )
    # hinge arms anchoring the door into the body slot below
    for j, sx in enumerate((-1.0, 1.0)):
        door.visual(
            Box((0.018, 0.055, 0.012)),
            origin=Origin(xyz=(sx * 0.20, 0.0275, 0.004)),
            material="dark_steel",
            name=f"hinge_arm_{j}",
        )

    model.articulation(
        f"body_to_door_{index}",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=1.5, lower=0.0, upper=math.pi / 2.0,
        ),
    )
    return door


def _add_rack(model, body, index: int, hinge_z: float):
    """Create one shelf rack part + prismatic articulation for oven unit *index*."""
    rack = model.part(f"shelf_rack_{index}")
    rack.visual(
        mesh_from_cadquery(_wire_rack(), f"rack_grid_{index}"),
        material="chrome_wire",
        name="rack_grid",
    )

    # Rack vertical offset from hinge: 0.1513 m (matches parent calibration)
    rack_z = hinge_z + 0.1513

    model.articulation(
        f"body_to_shelf_rack_{index}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(0.0, 0.03, rack_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.3, lower=0.0, upper=RACK_TRAVEL,
        ),
    )
    return rack


# ===================================================================== build

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_double_wall_oven")

    # ---- materials
    model.material("body_gray", rgba=(0.80, 0.80, 0.81, 1.0))
    model.material("door_gray", rgba=(0.62, 0.62, 0.64, 1.0))
    model.material("dark_glass", rgba=(0.07, 0.07, 0.09, 1.0))
    model.material("display_glass", rgba=(0.13, 0.14, 0.17, 1.0))
    model.material("display_digits", rgba=(0.92, 0.95, 0.99, 1.0))
    model.material("icon_print", rgba=(0.48, 0.50, 0.53, 1.0))
    model.material("frosted_glass", rgba=(0.92, 0.93, 0.94, 1.0))
    model.material("aluminum", rgba=(0.78, 0.79, 0.82, 1.0))
    model.material("cavity_dark", rgba=(0.22, 0.22, 0.23, 1.0))
    model.material("dark_steel", rgba=(0.34, 0.34, 0.36, 1.0))
    model.material("chrome_wire", rgba=(0.72, 0.73, 0.75, 1.0))

    # --------------------------------------------------------- oven body (root)
    body = model.part("oven_body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell"),
        material="body_gray",
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_front_fascia(), "front_fascia"),
        material="body_gray",
        name="front_fascia",
    )

    # one cavity liner per oven unit
    for i, hz in enumerate(OVEN_HINGE_Z):
        body.visual(
            mesh_from_cadquery(_cavity_liner(hz), f"cavity_liner_{i}"),
            material="cavity_dark",
            name=f"cavity_liner_{i}",
        )

    # rack support rails on each cavity side wall (two sides × N ovens)
    for i, hz in enumerate(OVEN_HINGE_Z):
        rail_z = hz + 0.141  # rail center sits just below the rack wire plane
        for j, sx in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.014, 0.40, 0.012)),
                origin=Origin(xyz=(sx * 0.233, 0.23, rail_z)),
                material="dark_steel",
                name=f"shelf_rail_{i}_{j}",
            )

    # ---- control strip (fixed, top of fascia)
    body.visual(
        Box((0.55, 0.004, 0.085)),
        origin=Origin(xyz=(0.0, -0.021, CTRL_Z_CENTER)),
        material="dark_glass",
        name="control_glass",
    )
    body.visual(
        Box((0.13, 0.002, 0.050)),
        origin=Origin(xyz=(0.0, -0.0235, CTRL_Z_CENTER)),
        material="display_glass",
        name="clock_display",
    )
    body.visual(
        Box((0.045, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.0252, CTRL_Z_CENTER)),
        material="display_digits",
        name="clock_digits",
    )
    icon_idx = 0
    for sx in (-1.0, 1.0):
        for x in (0.085, 0.115):
            for dz in (-0.0125, 0.0125):
                body.visual(
                    Box((0.016, 0.002, 0.016)),
                    origin=Origin(xyz=(sx * x, -0.0235, CTRL_Z_CENTER + dz)),
                    material="icon_print",
                    name=f"touch_icon_{icon_idx}",
                )
                icon_idx += 1

    # ------------------------------------------------ doors and racks (looped)
    for i, hz in enumerate(OVEN_HINGE_Z):
        _add_door(model, body, i, hz)
        _add_rack(model, body, i, hz)

    return model


# ===================================================================== tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")

    doors = [object_model.get_part(f"door_{i}") for i in range(N_OVENS)]
    racks = [object_model.get_part(f"shelf_rack_{i}") for i in range(N_OVENS)]
    hinges = [object_model.get_articulation(f"body_to_door_{i}") for i in range(N_OVENS)]
    slides = [object_model.get_articulation(f"body_to_shelf_rack_{i}") for i in range(N_OVENS)]

    # ---- intentional overlap allowances: hinge arms engage body slots
    for i, door in enumerate(doors):
        for arm in ("hinge_arm_0", "hinge_arm_1"):
            ctx.allow_overlap(
                door, body, elem_a=arm, elem_b="body_shell",
                reason=f"Door {i} hinge arm intentionally engages a slot in the body wall.",
            )
            ctx.allow_overlap(
                door, body, elem_a=arm, elem_b=f"cavity_liner_{i}",
                reason=f"Door {i} hinge arm passes through the liner front-bottom edge slot.",
            )
            ctx.allow_overlap(
                door, body, elem_a=arm, elem_b="front_fascia",
                reason=f"Door {i} hinge arm passes through a slot in the fascia border.",
            )

    # ---- overall envelope: ~0.60 wide, ~1.20 tall, ~0.55 deep, grounded
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "fascia is ~0.60 m wide and ~1.20 m tall, grounded at z=0",
        aabb is not None
        and abs((aabb[1][0] - aabb[0][0]) - FASCIA_W) < 0.01
        and abs(aabb[1][2] - FASCIA_H) < 0.02
        and abs(aabb[0][2]) < 0.005,
        details=f"body aabb={aabb}",
    )
    ctx.check(
        "body is ~0.55 m deep from fascia front to shell back",
        aabb is not None and abs((aabb[1][1] - aabb[0][1]) - TOTAL_D) < 0.01,
        details=f"body aabb={aabb}",
    )

    # ---- two oven doors exist, stacked vertically
    ctx.check(
        "two oven doors exist",
        len(doors) == 2,
        details=f"found {len(doors)} doors",
    )
    door0_aabb = ctx.part_world_aabb(doors[0])
    door1_aabb = ctx.part_world_aabb(doors[1])
    ctx.check(
        "door_1 is stacked above door_0",
        door0_aabb is not None and door1_aabb is not None
        and door1_aabb[0][2] > door0_aabb[1][2] - 0.02,
        details=f"door_0 aabb={door0_aabb}, door_1 aabb={door1_aabb}",
    )

    # ---- uniform hinge policy: every door revolute about +X at its hinge z
    for i, (hinge, hz) in enumerate(zip(hinges, OVEN_HINGE_Z)):
        ctx.check(
            f"door_{i} hinge is revolute about +X at z={hz:.3f}",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and tuple(hinge.axis) == (1.0, 0.0, 0.0)
            and abs(hinge.origin.xyz[2] - hz) < 1e-9
            and abs(hinge.motion_limits.lower) < 1e-9
            and abs(hinge.motion_limits.upper - math.pi / 2.0) < 1e-6,
            details=(
                f"axis={hinge.axis}, origin={hinge.origin.xyz}, "
                f"limits=({hinge.motion_limits.lower}, {hinge.motion_limits.upper})"
            ),
        )

    # ---- rack slides: prismatic along -Y with 0.35 m travel
    for i, slide in enumerate(slides):
        ctx.check(
            f"shelf_rack_{i} slide is prismatic along -Y with 0.35 m travel",
            slide.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide.axis) == (0.0, -1.0, 0.0)
            and abs(slide.motion_limits.upper - RACK_TRAVEL) < 1e-9,
            details=f"axis={slide.axis}, limits=({slide.motion_limits.lower}, {slide.motion_limits.upper})",
        )

    # ---- closed pose: each door seats in its fascia opening
    for i, door in enumerate(doors):
        ctx.expect_within(
            door, body, axes="xz",
            inner_elem="door_frame", outer_elem="front_fascia",
            margin=0.001,
            name=f"closed door_{i} stays within the fascia outline",
        )
        ctx.expect_gap(
            body, door, axis="y",
            positive_elem="body_shell", negative_elem="door_frame",
            min_gap=0.002, max_gap=0.012,
            name=f"closed door_{i} sits just in front of the body shell",
        )
        ctx.expect_within(
            door, door, axes="xz",
            inner_elem="window_glass", outer_elem="door_frame",
            margin=0.0,
            name=f"frosted window captured inside door_{i} frame",
        )

    # ---- control strip occupies the top of the fascia
    glass = ctx.part_element_world_aabb(body, elem="control_glass")
    disp = ctx.part_element_world_aabb(body, elem="clock_display")
    ctx.check(
        "dark touch glass and centered display sit at the top of the fascia",
        glass is not None
        and disp is not None
        and glass[0][2] > 1.05
        and disp[0][2] > 1.05
        and abs((disp[0][0] + disp[1][0]) / 2.0) < 0.005,
        details=f"glass={glass}, display={disp}",
    )

    # ---- open pose: each door independently swings to horizontal
    for i, (door, hinge, hz) in enumerate(zip(doors, hinges, OVEN_HINGE_Z)):
        bar_closed = ctx.part_element_world_aabb(door, elem="handle_bar")
        bar_closed_z = (bar_closed[0][2] + bar_closed[1][2]) / 2.0 if bar_closed else None
        with ctx.pose({hinge: math.pi / 2.0}):
            door_open = ctx.part_world_aabb(door)
            bar_open = ctx.part_element_world_aabb(door, elem="handle_bar")
            ctx.check(
                f"door_{i} open lies horizontal, above the floor",
                door_open is not None
                and door_open[1][2] < hz + 0.15
                and door_open[0][1] < -0.40
                and door_open[0][2] > 0.0,
                details=f"open door aabb={door_open}",
            )
            ctx.check(
                f"door_{i} handle bar drops near the hinge level when open",
                bar_open is not None
                and bar_closed_z is not None
                and bar_closed_z > hz + 0.30
                and (bar_open[0][2] + bar_open[1][2]) / 2.0 < hz + 0.08,
                details=f"closed_z={bar_closed_z}, open bar={bar_open}",
            )

    # ---- racks: housed inside body, slide out -Y with retained insertion
    for i, (rack, slide, hz) in enumerate(zip(racks, slides, OVEN_HINGE_Z)):
        ctx.expect_within(
            rack, body, axes="xy", margin=0.0,
            name=f"closed shelf_rack_{i} is fully housed inside the body",
        )
        ctx.expect_gap(
            rack, body, axis="z",
            negative_elem=f"shelf_rail_{i}_0",
            max_penetration=0.001, max_gap=0.003,
            name=f"shelf_rack_{i} wires rest on the cavity side rails",
        )
        rack_rest = ctx.part_world_position(rack)
        with ctx.pose({slide: RACK_TRAVEL}):
            rack_out = ctx.part_world_position(rack)
            ctx.expect_overlap(
                rack, body, axes="y", min_overlap=0.04,
                name=f"extended shelf_rack_{i} keeps retained insertion",
            )
        ctx.check(
            f"shelf_rack_{i} pulls out 0.35 m toward the user (-Y)",
            rack_rest is not None
            and rack_out is not None
            and abs((rack_rest[1] - rack_out[1]) - RACK_TRAVEL) < 1e-6,
            details=f"rest={rack_rest}, out={rack_out}",
        )

    return ctx.report()


object_model = build_object_model()
