from __future__ import annotations

"""Built-in single electric wall oven with a drop-down door and two slide-out racks.

Articraft brief:
- Object: built-in single electric wall oven, fascia 0.60 m wide x 0.60 m tall,
  body 0.55 m deep overall (hollow box recessed behind the front fascia).
- Root/support: oven_body (hollow shell + front fascia + control strip),
  grounded at z=0; +Y points into the cabinetry, -Y is the user-facing front.
- Parts: oven_body (root), door (drop-down, revolute on the bottom edge),
  rack_0 and rack_1 (two wire racks on independent prismatic slides along the
  depth axis, evenly spaced vertically inside the cavity).
- Articulations:
  * body_to_door: REVOLUTE, hinge along the door's bottom front edge,
    axis +X so positive q swings the top of the door outward (-Y) and down,
    limits 0 (closed vertical) .. pi/2 (open horizontal).
  * body_to_rack_0, body_to_rack_1: PRISMATIC, axis (0,-1,0) so positive q
    pulls each rack out of the cavity toward the user, travel 0.35 m with
    retained insertion.
- Visible geometry: matte light-gray steel fascia and shell, dark glass touch
  strip with centered clock display and flanking touch icons (top fifth),
  gray door frame with large frosted-white glass window, brushed-aluminum bar
  handle on two posts near the door top, dark hollow cavity with side rails.
- Intentional overlaps: door hinge arms embed into the body bottom slot
  (scoped allowances); layered control-strip visuals embed within one part.
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
FASCIA_H = 0.60
FASCIA_T = 0.022  # y: -0.020 .. +0.002 (embeds 2 mm into the shell front)
BODY_W = 0.56
BODY_H = 0.56  # z: 0.02 .. 0.58
BODY_D = 0.53  # y: 0.00 .. 0.53
TOTAL_D = 0.55  # fascia front (-0.02) to shell back (0.53)

OPEN_W = 0.565  # fascia door opening
OPEN_Z0 = 0.058
OPEN_Z1 = 0.490

DOOR_W = 0.555
DOOR_T = 0.040
DOOR_H = 0.425  # local z 0.003 .. 0.4285
HINGE_Z = OPEN_Z0  # hinge along the door's bottom front edge
HINGE_Y = -0.045  # door front face plane

RACK_TRAVEL = 0.35
RACK_COUNT = 2

# Cavity inner vertical range: liner inner box bottom z=0.075, top z=0.445
# Even spacing: divide into (RACK_COUNT+1) equal zones
_CAVITY_Z0 = 0.075
_CAVITY_Z1 = 0.445
_RACK_ZONE = (_CAVITY_Z1 - _CAVITY_Z0) / (RACK_COUNT + 1)
RACK_Z = [_CAVITY_Z0 + _RACK_ZONE * (i + 1) for i in range(RACK_COUNT)]


def _cyl(radius: float, length: float, axis: str, center: tuple[float, float, float]):
    wp = cq.Workplane("XY").cylinder(length, radius)  # long axis Z
    if axis == "x":
        wp = wp.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        wp = wp.rotate((0, 0, 0), (1, 0, 0), 90)
    return wp.translate(center)


def _body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0, 0.265, 0.30))
    cavity = cq.Workplane("XY").box(0.50, 0.51, 0.40).translate((0, 0.245, 0.26))
    return outer.cut(cavity)


def _front_fascia() -> cq.Workplane:
    plate = cq.Workplane("XY").box(FASCIA_W, FASCIA_T, FASCIA_H).translate((0, -0.009, 0.30))
    opening = (
        cq.Workplane("XY")
        .box(OPEN_W, 0.08, OPEN_Z1 - OPEN_Z0)
        .translate((0, -0.009, (OPEN_Z0 + OPEN_Z1) / 2.0))
    )
    return plate.cut(opening)


def _cavity_liner() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.504, 0.502, 0.404).translate((0, 0.247, 0.26))
    inner = cq.Workplane("XY").box(0.470, 0.530, 0.370).translate((0, 0.215, 0.26))
    return outer.cut(inner)


def _door_frame() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0, DOOR_T / 2.0, 0.003 + DOOR_H / 2.0))
    )
    window = cq.Workplane("XY").box(0.42, 0.08, 0.30).translate((0, DOOR_T / 2.0, 0.185))
    return panel.cut(window)


def _wire_rack() -> cq.Workplane:
    rack = _cyl(0.0045, 0.40, "y", (0.230, 0.20, 0.0))
    rack = rack.union(_cyl(0.0045, 0.40, "y", (-0.230, 0.20, 0.0)))
    # front / mid / back cross bars (X direction)
    for y in (0.006, 0.20, 0.394):
        rack = rack.union(_cyl(0.004, 0.464, "x", (0.0, y, 0.0)))
    # longitudinal grill wires (Y direction)
    for i in range(9):
        x = -0.184 + 0.046 * i
        rack = rack.union(_cyl(0.003, 0.388, "y", (x, 0.20, 0.0)))
    return rack


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_single_wall_oven")

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

    # ------------------------------------------------------------- oven body
    body = model.part("oven_body")
    body.visual(mesh_from_cadquery(_body_shell(), "body_shell"), material="body_gray", name="body_shell")
    body.visual(
        mesh_from_cadquery(_front_fascia(), "front_fascia"),
        material="body_gray",
        name="front_fascia",
    )
    body.visual(
        mesh_from_cadquery(_cavity_liner(), "cavity_liner"),
        material="cavity_dark",
        name="cavity_liner",
    )
    # rack support rails on the cavity side walls (2 heights x 2 sides)
    for ri, rz in enumerate(RACK_Z):
        for si, sx in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.014, 0.40, 0.012)),
                origin=Origin(xyz=(sx * 0.233, 0.23, rz - 0.010)),
                material="dark_steel",
                name=f"shelf_rail_{ri}_{si}",
            )

    # control strip (fixed, top fifth of the fascia)
    body.visual(
        Box((0.55, 0.004, 0.085)),
        origin=Origin(xyz=(0.0, -0.021, 0.5425)),
        material="dark_glass",
        name="control_glass",
    )
    body.visual(
        Box((0.13, 0.002, 0.050)),
        origin=Origin(xyz=(0.0, -0.0235, 0.5425)),
        material="display_glass",
        name="clock_display",
    )
    body.visual(
        Box((0.045, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.0252, 0.5425)),
        material="display_digits",
        name="clock_digits",
    )
    icon_idx = 0
    for sx in (-1.0, 1.0):
        for x in (0.085, 0.115):
            for z in (0.530, 0.555):
                body.visual(
                    Box((0.016, 0.002, 0.016)),
                    origin=Origin(xyz=(sx * x, -0.0235, z)),
                    material="icon_print",
                    name=f"touch_icon_{icon_idx}",
                )
                icon_idx += 1

    # ------------------------------------------------------------------ door
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "door_frame"),
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
    for i, sx in enumerate((-1.0, 1.0)):
        door.visual(
            Cylinder(radius=0.009, length=0.036),
            origin=Origin(xyz=(sx * 0.185, -0.014, 0.392), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="aluminum",
            name=f"handle_post_{i}",
        )
    # hinge arms anchoring the door into the body bottom slot
    for i, sx in enumerate((-1.0, 1.0)):
        door.visual(
            Box((0.018, 0.055, 0.012)),
            origin=Origin(xyz=(sx * 0.20, 0.0275, 0.004)),
            material="dark_steel",
            name=f"hinge_arm_{i}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=math.pi / 2.0),
    )

    # -------------------------------------------------------- slide-out racks
    racks = []
    rack_slides = []
    for i in range(RACK_COUNT):
        rack = model.part(f"rack_{i}")
        rack.visual(
            mesh_from_cadquery(_wire_rack(), f"rack_grid_{i}"),
            material="chrome_wire",
            name="rack_grid",
        )
        racks.append(rack)

        slide = model.articulation(
            f"body_to_rack_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=rack,
            origin=Origin(xyz=(0.0, 0.03, RACK_Z[i])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=0.3, lower=0.0, upper=RACK_TRAVEL),
        )
        rack_slides.append(slide)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    racks = [object_model.get_part(f"rack_{i}") for i in range(RACK_COUNT)]
    slides = [object_model.get_articulation(f"body_to_rack_{i}") for i in range(RACK_COUNT)]

    # intentional embeddings: hinge arms anchor into the body bottom slot
    for arm in ("hinge_arm_0", "hinge_arm_1"):
        ctx.allow_overlap(
            door,
            body,
            elem_a=arm,
            elem_b="body_shell",
            reason="Door hinge arm intentionally engages a slot in the body bottom wall.",
        )
        ctx.allow_overlap(
            door,
            body,
            elem_a=arm,
            elem_b="cavity_liner",
            reason="Door hinge arm passes through the liner front-bottom edge slot.",
        )
        ctx.allow_overlap(
            door,
            body,
            elem_a=arm,
            elem_b="front_fascia",
            reason="Door hinge arm passes through a slot in the fascia bottom border.",
        )

    # ---- overall envelope: ~0.60 wide x 0.60 tall, 0.55 deep body, grounded
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "fascia is ~0.60 m wide and ~0.60 m tall, grounded at z=0",
        aabb is not None
        and abs((aabb[1][0] - aabb[0][0]) - FASCIA_W) < 0.01
        and abs(aabb[1][2] - FASCIA_H) < 0.01
        and abs(aabb[0][2]) < 0.005,
        details=f"body aabb={aabb}",
    )
    ctx.check(
        "body is ~0.55 m deep from fascia front to shell back",
        aabb is not None and abs((aabb[1][1] - aabb[0][1]) - TOTAL_D) < 0.01,
        details=f"body aabb={aabb}",
    )

    # ---- door joint contract: bottom-edge hinge, axis along width, 0..90 deg
    ctx.check(
        "door hinge is revolute about +X (oven width) at the door bottom edge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (1.0, 0.0, 0.0)
        and abs(hinge.origin.xyz[2] - HINGE_Z) < 1e-9
        and abs(hinge.motion_limits.lower) < 1e-9
        and abs(hinge.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=f"axis={hinge.axis}, origin={hinge.origin.xyz}, limits=({hinge.motion_limits.lower}, {hinge.motion_limits.upper})",
    )
    for i, slide in enumerate(slides):
        ctx.check(
            f"rack_{i} slide is prismatic along -Y (depth axis) with 0.35 m travel",
            slide.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide.axis) == (0.0, -1.0, 0.0)
            and abs(slide.motion_limits.upper - RACK_TRAVEL) < 1e-9,
            details=f"axis={slide.axis}, limits=({slide.motion_limits.lower}, {slide.motion_limits.upper})",
        )

    # ---- closed pose: door seats inside the fascia opening, in front of shell
    ctx.expect_within(
        door,
        body,
        axes="xz",
        inner_elem="door_frame",
        outer_elem="front_fascia",
        margin=0.001,
        name="closed door panel stays within the fascia outline",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="body_shell",
        negative_elem="door_frame",
        min_gap=0.002,
        max_gap=0.012,
        name="closed door sits just in front of the body shell",
    )
    ctx.expect_within(
        door,
        door,
        axes="xz",
        inner_elem="window_glass",
        outer_elem="door_frame",
        margin=0.0,
        name="frosted window glass is captured inside the door frame",
    )

    # handle bar stands proud of the door face on its posts
    bar = ctx.part_element_world_aabb(door, elem="handle_bar")
    frame = ctx.part_element_world_aabb(door, elem="door_frame")
    ctx.check(
        "handle bar stands off in front of the door face near its top edge",
        bar is not None
        and frame is not None
        and bar[1][1] < frame[0][1] - 0.02
        and bar[0][2] > frame[1][2] - 0.12,
        details=f"bar={bar}, frame={frame}",
    )

    # control strip occupies the top fifth of the fascia
    glass = ctx.part_element_world_aabb(body, elem="control_glass")
    disp = ctx.part_element_world_aabb(body, elem="clock_display")
    ctx.check(
        "dark touch glass and centered display sit in the top fifth of the fascia",
        glass is not None
        and disp is not None
        and glass[0][2] > 0.48
        and disp[0][2] > 0.48
        and abs((disp[0][0] + disp[1][0]) / 2.0) < 0.005,
        details=f"glass={glass}, display={disp}",
    )

    # ---- open pose: door swings outward and down to horizontal at 90 deg.
    # The off-axis handle bar (standing proud of the door face) must travel
    # from near the fascia top to just above the floor, proving the rotation.
    bar_closed_z = (bar[0][2] + bar[1][2]) / 2.0 if bar is not None else None
    with ctx.pose({hinge: math.pi / 2.0}):
        door_open = ctx.part_world_aabb(door)
        bar_open = ctx.part_element_world_aabb(door, elem="handle_bar")
        ctx.check(
            "open door lies horizontal in front of the oven, above the floor",
            door_open is not None
            and door_open[1][2] < 0.13
            and door_open[0][1] < -0.40
            and door_open[0][2] > 0.0,
            details=f"open door aabb={door_open}",
        )
        ctx.check(
            "off-axis handle bar drops from fascia-top height to near the floor",
            bar_open is not None
            and bar_closed_z is not None
            and bar_closed_z > 0.40
            and (bar_open[0][2] + bar_open[1][2]) / 2.0 < 0.05,
            details=f"closed_z={bar_closed_z}, open bar={bar_open}",
        )

    # ---- racks: rest on the side rails inside the cavity, slide out -Y
    for i, (rack, slide) in enumerate(zip(racks, slides)):
        ctx.expect_within(
            rack,
            body,
            axes="xy",
            margin=0.0,
            name=f"closed rack_{i} is fully housed inside the oven body",
        )
        ctx.expect_gap(
            rack,
            body,
            axis="z",
            negative_elem=f"shelf_rail_{i}_0",
            max_penetration=0.001,
            max_gap=0.001,
            name=f"rack_{i} wires rest on the cavity side rails",
        )
        rack_rest = ctx.part_world_position(rack)
        with ctx.pose({slide: RACK_TRAVEL}):
            rack_out = ctx.part_world_position(rack)
            ctx.expect_overlap(
                rack,
                body,
                axes="y",
                min_overlap=0.04,
                name=f"extended rack_{i} keeps retained insertion in the cavity",
            )
        ctx.check(
            f"rack_{i} pulls out 0.35 m toward the user (-Y)",
            rack_rest is not None
            and rack_out is not None
            and abs((rack_rest[1] - rack_out[1]) - RACK_TRAVEL) < 1e-6,
            details=f"rest={rack_rest}, out={rack_out}",
        )

    # ---- racks are at distinct heights (even vertical spacing)
    rack_centers = [ctx.part_world_position(r) for r in racks]
    ctx.check(
        "two racks sit at distinct heights inside the cavity",
        all(c is not None for c in rack_centers)
        and len(rack_centers) == RACK_COUNT
        and abs(rack_centers[1][2] - rack_centers[0][2]) > 0.08,
        details=f"rack_centers={rack_centers}",
    )

    return ctx.report()


object_model = build_object_model()
