from __future__ import annotations

"""Freestanding stainless-steel dishwasher.

Articraft brief:
- Object: freestanding dishwasher, ~0.60 m wide x 0.62 m deep x 0.85 m tall.
- Root/support: hollow cabinet (open front) on a recessed dark kick plinth,
  with a slightly overhanging brushed countertop-style top slab.
- Parts: cabinet (root), front door (bottom-hinged revolute), upper wire rack
  (prismatic), lower wire rack (prismatic).
- Articulations:
  * door_hinge: REVOLUTE at the bottom front edge, axis -X so positive q folds
    the door forward/down from vertical (q=0) to horizontal (q=pi/2).
  * upper_rack_slide / lower_rack_slide: PRISMATIC along +Y (depth axis),
    0..0.45 m of forward travel out of the tub.
- Visible geometry: brushed silver shell, polished stainless tub liner,
  chrome wire racks, dark control strip on the door top edge with pocket
  handle, round buttons, blue LCD.
- Frame: X = width, Y = depth (front = +Y), Z = up; grounded at z = 0.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------- dimensions
TOP_W = 0.60  # top slab width
TOP_D = 0.62  # top slab depth
TOTAL_H = 0.85  # overall height

BODY_W = 0.58  # cabinet body width  (x in [-0.29, 0.29])
BODY_FRONT_Y = 0.27  # cabinet body front face
BODY_BACK_Y = -0.29  # cabinet body back face
WALL_T = 0.02

PLINTH_H = 0.07
BODY_TOP_Z = 0.82  # underside of the top slab
SLAB_T = 0.03

DOOR_T = 0.022
SLAB_BOTTOM_Z = BODY_TOP_Z - 0.005  # slab visual dips 5 mm below BODY_TOP_Z
DOOR_H = SLAB_BOTTOM_Z - PLINTH_H - 0.002  # 0.743, clears the slab underside
HINGE_Z = PLINTH_H

RACK_W = 0.50
RACK_D = 0.48
WIRE = 0.006
RACK_TRAVEL = 0.45
TUB_FLOOR_TOP = 0.095  # top surface of the polished tub floor liner
WHEEL_R = 0.012
LOWER_RACK_Z = TUB_FLOOR_TOP + 0.010  # rack basket floor above wheel axles
UPPER_RACK_Z = 0.45  # also the top surface of the upper rack rails

# ---------------------------------------------------------------- materials
BRUSHED_STEEL = Material(name="brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
STEEL_TOP = Material(name="steel_top", rgba=(0.66, 0.67, 0.69, 1.0))
POLISHED_TUB = Material(name="polished_tub", rgba=(0.82, 0.83, 0.85, 1.0))
PLINTH_GRAY = Material(name="plinth_gray", rgba=(0.28, 0.29, 0.30, 1.0))
STRIP_GRAY = Material(name="strip_gray", rgba=(0.20, 0.21, 0.23, 1.0))
HANDLE_DARK = Material(name="handle_dark", rgba=(0.10, 0.10, 0.11, 1.0))
LCD_BLUE = Material(name="lcd_blue", rgba=(0.25, 0.50, 0.92, 1.0))
BUTTON_GRAY = Material(name="button_gray", rgba=(0.55, 0.56, 0.58, 1.0))
CHROME_WIRE = Material(name="chrome_wire", rgba=(0.85, 0.86, 0.88, 1.0))


def _linspace(lo: float, hi: float, n: int) -> list[float]:
    if n == 1:
        return [(lo + hi) * 0.5]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


def _build_cabinet(model: ArticulatedObject):
    cab = model.part("cabinet")

    # Recessed dark kick plinth (slightly overlaps the cabinet floor above
    # so the part is one connected body).
    cab.visual(
        Box((0.52, 0.52, PLINTH_H + 0.005)),
        origin=Origin(xyz=(0.0, -0.02, (PLINTH_H + 0.005) / 2)),
        material=PLINTH_GRAY,
        name="kick_plinth",
    )

    wall_h = BODY_TOP_Z - PLINTH_H  # 0.75
    wall_zc = PLINTH_H + wall_h / 2  # 0.445
    body_d = BODY_FRONT_Y - BODY_BACK_Y  # 0.56
    body_yc = (BODY_FRONT_Y + BODY_BACK_Y) / 2  # -0.01

    # Hollow shell: two side walls, back wall, floor, ceiling. Front is open.
    for sx, tag in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        cab.visual(
            Box((WALL_T, body_d, wall_h)),
            origin=Origin(xyz=(sx * (BODY_W / 2 - WALL_T / 2), body_yc, wall_zc)),
            material=BRUSHED_STEEL,
            name=tag,
        )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, WALL_T, wall_h)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + WALL_T / 2, wall_zc)),
        material=BRUSHED_STEEL,
        name="back_wall",
    )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, body_d, WALL_T)),
        origin=Origin(xyz=(0.0, body_yc, PLINTH_H + WALL_T / 2)),
        material=BRUSHED_STEEL,
        name="cabinet_floor",
    )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, body_d, WALL_T)),
        origin=Origin(xyz=(0.0, body_yc, BODY_TOP_Z - WALL_T / 2)),
        material=BRUSHED_STEEL,
        name="cabinet_ceiling",
    )

    # Polished stainless tub liner, slightly embedded into the shell walls so
    # everything stays one connected body.
    liner_t = 0.008
    liner_h = 0.68
    liner_zc = PLINTH_H + WALL_T + liner_h / 2 - 0.004
    tub_back = BODY_BACK_Y + WALL_T  # -0.27 inner face of back wall
    liner_d = BODY_FRONT_Y - tub_back - 0.01  # stop just shy of the opening
    liner_yc = tub_back + liner_d / 2 - 0.002
    for sx, tag in ((-1.0, "tub_wall_0"), (1.0, "tub_wall_1")):
        cab.visual(
            Box((liner_t, liner_d, liner_h)),
            origin=Origin(
                xyz=(sx * (BODY_W / 2 - WALL_T - liner_t / 2 + 0.002), liner_yc, liner_zc)
            ),
            material=POLISHED_TUB,
            name=tag,
        )
    cab.visual(
        Box((BODY_W - 2 * WALL_T, liner_t, liner_h)),
        origin=Origin(xyz=(0.0, tub_back + liner_t / 2 - 0.002, liner_zc)),
        material=POLISHED_TUB,
        name="tub_back_panel",
    )
    cab.visual(
        Box((BODY_W - 2 * WALL_T - 0.01, liner_d, liner_t)),
        origin=Origin(xyz=(0.0, liner_yc, TUB_FLOOR_TOP - liner_t / 2)),
        material=POLISHED_TUB,
        name="tub_floor",
    )

    # Upper-rack slide rails on the tub side walls (slightly embedded into the
    # liner walls so they stay connected). Rail top surface = UPPER_RACK_Z.
    tub_inner_x = BODY_W / 2 - WALL_T - liner_t + 0.002  # liner inner face
    for sx, tag in ((-1.0, "rack_rail_0"), (1.0, "rack_rail_1")):
        cab.visual(
            Box((0.014, 0.50, 0.008)),
            origin=Origin(xyz=(sx * (tub_inner_x - 0.005), -0.01, UPPER_RACK_Z - 0.004)),
            material=POLISHED_TUB,
            name=tag,
        )

    # Overhanging brushed countertop-style top slab.
    cab.visual(
        Box((TOP_W, TOP_D, SLAB_T + 0.005)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + (SLAB_T - 0.005) / 2)),
        material=STEEL_TOP,
        name="top_slab",
    )
    return cab


def _build_door(model: ArticulatedObject):
    """Door authored in its hinge frame: hinge axis through local origin,
    panel vertical at q=0, extending +Z (up) and +Y (forward) in thickness."""
    door = model.part("front_door")

    door.visual(
        Box((BODY_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(0.0, DOOR_T / 2, DOOR_H / 2)),
        material=BRUSHED_STEEL,
        name="door_panel",
    )

    # Dark control strip across the top edge, slightly proud of the panel.
    strip_h = 0.07
    strip_zc = DOOR_H - 0.005 - strip_h / 2  # 0.71
    door.visual(
        Box((BODY_W, 0.010, strip_h)),
        origin=Origin(xyz=(0.0, DOOR_T + 0.003, strip_zc)),
        material=STRIP_GRAY,
        name="control_strip",
    )
    strip_face = DOOR_T + 0.008  # front face of the strip

    # Proud full-width bar handle standing off the door face.
    # Two rectangular standoff brackets mount the bar to the door panel,
    # positioned just below the control strip centerline.
    bar_r = 0.010
    bar_len = 0.50
    standoff_depth = 0.032
    standoff_w = 0.028
    standoff_h = 0.022
    bar_z = strip_zc - 0.008  # just below strip center
    bar_y_center = DOOR_T + standoff_depth
    for i, sx in enumerate((-1.0, 1.0)):
        door.visual(
            Box((standoff_w, standoff_depth, standoff_h)),
            origin=Origin(
                xyz=(sx * 0.20, DOOR_T + standoff_depth / 2, bar_z)
            ),
            material=BRUSHED_STEEL,
            name=f"handle_standoff_{i}",
        )
    door.visual(
        Cylinder(radius=bar_r, length=bar_len),
        origin=Origin(
            xyz=(0.0, bar_y_center, bar_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=BRUSHED_STEEL,
        name="bar_handle",
    )

    # Blue LCD display right of center.
    door.visual(
        Box((0.085, 0.004, 0.024)),
        origin=Origin(xyz=(0.155, strip_face + 0.001, strip_zc + 0.004)),
        material=LCD_BLUE,
        name="display_lcd",
    )

    # Small round buttons left of center (cylinders along +Y).
    for i, bx in enumerate((-0.215, -0.180, -0.145, -0.110)):
        door.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(
                xyz=(bx, strip_face, strip_zc + 0.004),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=BUTTON_GRAY,
            name=f"button_{i}",
        )
    return door


def _populate_rack(rack, *, height: float, support: str) -> None:
    """Open wire-frame basket in the rack's local frame.

    Local frame: origin at bottom center; x in [-RACK_W/2, RACK_W/2],
    y in [-RACK_D/2, RACK_D/2], z in [0, height].

    support: "wheels" (lower rack, rolls on the tub floor) or "runners"
    (upper rack, glide bars resting on the cabinet rack rails).
    """
    hw = RACK_W / 2
    hd = RACK_D / 2

    if support == "wheels":
        # Four roller wheels under the basket; their bottoms rest on the tub
        # floor (world TUB_FLOOR_TOP). Axles along X.
        wheel_zc = (TUB_FLOOR_TOP + WHEEL_R) - LOWER_RACK_Z  # local
        idx = 0
        for wy in (-0.20, 0.20):
            for wx in (-0.18, 0.18):
                rack.visual(
                    Cylinder(radius=WHEEL_R, length=0.010),
                    origin=Origin(xyz=(wx, wy, wheel_zc), rpy=(0.0, math.pi / 2, 0.0)),
                    material=BUTTON_GRAY,
                    name=f"wheel_{idx}",
                )
                idx += 1
    else:
        # Side glide runners that rest on the cabinet rack rails (rail top is
        # at the rack's local z = 0).
        for k, sx in enumerate((-1.0, 1.0)):
            rack.visual(
                Box((0.011, RACK_D, 0.006)),
                origin=Origin(xyz=(sx * 0.2535, 0.0, 0.003)),
                material=CHROME_WIRE,
                name=f"side_runner_{k}",
            )

    # Floor: longitudinal rods (along Y) crossed by lateral rods (along X).
    for i, x in enumerate(_linspace(-hw + 0.01, hw - 0.01, 9)):
        rack.visual(
            Box((WIRE, RACK_D - 0.004, WIRE)),
            origin=Origin(xyz=(x, 0.0, 0.003)),
            material=CHROME_WIRE,
            name=f"floor_rod_y_{i}",
        )
    cross_ys = _linspace(-hd + 0.003, hd - 0.003, 5)
    for i, y in enumerate(cross_ys):
        rack.visual(
            Box((RACK_W, WIRE, WIRE)),
            origin=Origin(xyz=(0.0, y, 0.007)),
            material=CHROME_WIRE,
            name=f"floor_rod_x_{i}",
        )

    # Top perimeter rail.
    rail_z = height - WIRE / 2
    for i, y in enumerate((-hd + 0.003, hd - 0.003)):
        rack.visual(
            Box((RACK_W, WIRE, WIRE)),
            origin=Origin(xyz=(0.0, y, rail_z)),
            material=CHROME_WIRE,
            name=f"top_rail_xy_{i}",
        )
    for i, x in enumerate((-hw + 0.003, hw - 0.003)):
        rack.visual(
            Box((WIRE, RACK_D, WIRE)),
            origin=Origin(xyz=(x, 0.0, rail_z)),
            material=CHROME_WIRE,
            name=f"top_rail_side_{i}",
        )

    # Vertical wires: sides aligned with the floor cross rods so every wire
    # lands on another wire (no floating wire segments).
    for j, y in enumerate(cross_ys):
        for k, x in enumerate((-hw + 0.003, hw - 0.003)):
            rack.visual(
                Box((WIRE, WIRE, height)),
                origin=Origin(xyz=(x, y, height / 2)),
                material=CHROME_WIRE,
                name=f"side_wire_{k}_{j}",
            )
    # Front/back vertical wires aligned with longitudinal floor rods.
    for j, x in enumerate(_linspace(-hw + 0.01, hw - 0.01, 7)):
        for k, y in enumerate((-hd + 0.003, hd - 0.003)):
            rack.visual(
                Box((WIRE, WIRE, height)),
                origin=Origin(xyz=(x, y, height / 2)),
                material=CHROME_WIRE,
                name=f"face_wire_{k}_{j}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_dishwasher")

    cabinet = _build_cabinet(model)
    door = _build_door(model)

    upper_rack = model.part("upper_rack")
    _populate_rack(upper_rack, height=0.12, support="runners")
    lower_rack = model.part("lower_rack")
    _populate_rack(lower_rack, height=0.13, support="wheels")

    # Door: bottom-front horizontal hinge. Axis -X so positive q tips the top
    # of the door forward (+Y) and down, from vertical (0) to horizontal (~90deg).
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=math.pi / 2),
    )

    # Racks slide forward (+Y, out of the open front) on prismatic joints.
    rack_yc = -0.01  # rack center depth inside the tub
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, rack_yc, UPPER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=RACK_TRAVEL),
    )
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, rack_yc, LOWER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=RACK_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("front_door")
    upper_rack = object_model.get_part("upper_rack")
    lower_rack = object_model.get_part("lower_rack")
    hinge = object_model.get_articulation("door_hinge")
    upper_slide = object_model.get_articulation("upper_rack_slide")
    lower_slide = object_model.get_articulation("lower_rack_slide")

    # --- overall envelope: ~0.60 x 0.62 x 0.85, grounded at z = 0
    aabb = ctx.part_world_aabb(cabinet)
    ok = aabb is not None
    if ok:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ok = (
            abs((x1 - x0) - TOP_W) < 0.02
            and abs((y1 - y0) - TOP_D) < 0.02
            and abs(z1 - TOTAL_H) < 0.02
            and abs(z0) < 1e-6
        )
    ctx.check(
        "cabinet has ~0.60 x 0.62 x 0.85 envelope grounded at z=0",
        ok,
        details=f"cabinet aabb = {aabb}",
    )

    # --- joint plan: types, axes, limits
    ctx.check(
        "door hinge is revolute about a horizontal X axis, 0..~90deg",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(hinge.axis[0]) - 1.0) < 1e-9
        and abs(hinge.axis[1]) < 1e-9
        and abs(hinge.axis[2]) < 1e-9
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(hinge.motion_limits.upper - math.pi / 2) < 1e-6,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "hinge sits at the bottom front edge of the cabinet",
        abs(hinge.origin.xyz[1] - BODY_FRONT_Y) < 1e-9
        and abs(hinge.origin.xyz[2] - PLINTH_H) < 1e-9,
        details=f"hinge origin = {hinge.origin.xyz}",
    )
    for slide in (upper_slide, lower_slide):
        ctx.check(
            f"{slide.name} is prismatic along +Y with 0..0.45 m travel",
            slide.articulation_type == ArticulationType.PRISMATIC
            and abs(slide.axis[1] - 1.0) < 1e-9
            and abs(slide.axis[0]) < 1e-9
            and abs(slide.axis[2]) < 1e-9
            and slide.motion_limits is not None
            and abs(slide.motion_limits.lower - 0.0) < 1e-9
            and abs(slide.motion_limits.upper - RACK_TRAVEL) < 1e-9,
            details=f"axis={slide.axis}, limits={slide.motion_limits}",
        )
    ctx.check(
        "upper rack is mounted above the lower rack",
        upper_slide.origin.xyz[2] > lower_slide.origin.xyz[2] + 0.25,
        details=f"upper z={upper_slide.origin.xyz[2]}, lower z={lower_slide.origin.xyz[2]}",
    )

    # --- closed pose: door vertical, flush against the cabinet front
    panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ok = panel_aabb is not None
    if ok:
        (dx0, dy0, dz0), (dx1, dy1, dz1) = panel_aabb
        ok = (
            (dz1 - dz0) > 0.70  # tall (vertical) in closed pose
            and (dy1 - dy0) < 0.05  # thin in depth
            and dy0 >= BODY_FRONT_Y - 1e-6  # entirely in front of the tub
            and abs(dz0 - PLINTH_H) < 0.005  # starts at the plinth top
        )
    ctx.check(
        "closed door is a vertical full-height panel at the cabinet front",
        ok,
        details=f"door panel aabb = {panel_aabb}",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="door_panel",
        elem_b="side_wall_0",
        contact_tol=1e-4,
        name="closed door panel seats flush on the cabinet front face",
    )

    # control strip features live on the upper part of the closed door
    lcd_aabb = ctx.part_element_world_aabb(door, elem="display_lcd")
    handle_aabb = ctx.part_element_world_aabb(door, elem="bar_handle")
    ok = lcd_aabb is not None and handle_aabb is not None
    if ok:
        ok = (
            lcd_aabb[0][2] > 0.68
            and handle_aabb[0][2] > 0.68
            and lcd_aabb[0][1] > BODY_FRONT_Y + DOOR_T - 1e-6  # proud of the panel
        )
    ctx.check(
        "LCD and bar handle sit on the door top control strip, proud of the panel",
        ok,
        details=f"lcd={lcd_aabb}, handle={handle_aabb}",
    )
    # Bar handle is full-width (spans most of the door) and stands off the face
    if handle_aabb is not None:
        (hx0, hy0, hz0), (hx1, hy1, hz1) = handle_aabb
        handle_span = hx1 - hx0
        handle_standoff = hy0 - BODY_FRONT_Y
        ctx.check(
            "bar handle is full-width (>0.40 m span) and stands off the door face (>0.02 m)",
            handle_span > 0.40 and handle_standoff > 0.02,
            details=f"span={handle_span:.3f}, standoff={handle_standoff:.3f}",
        )
    # off-axis feature: buttons sit left of the door centerline, so opening
    # the door visibly carries them forward and down with the panel
    btn_aabb = ctx.part_element_world_aabb(door, elem="button_0")
    ctx.check(
        "round buttons sit off-axis on the left of the control strip",
        btn_aabb is not None and btn_aabb[1][0] < -0.10,
        details=f"button_0 aabb = {btn_aabb}",
    )

    # --- racks nest inside the hollow tub in closed pose
    for rack, tag in ((upper_rack, "upper"), (lower_rack, "lower")):
        ctx.expect_within(
            rack,
            cabinet,
            axes="xy",
            margin=0.0,
            name=f"{tag} rack stays inside the cabinet footprint when stowed",
        )
        ctx.expect_gap(
            door,
            rack,
            axis="y",
            min_gap=0.005,
            name=f"closed door clears the stowed {tag} rack",
        )
    ctx.expect_contact(
        lower_rack,
        cabinet,
        elem_a="wheel_0",
        elem_b="tub_floor",
        contact_tol=1e-4,
        name="lower rack wheels rest on the tub floor",
    )
    ctx.expect_contact(
        upper_rack,
        cabinet,
        elem_a="side_runner_0",
        elem_b="rack_rail_0",
        contact_tol=1e-4,
        name="upper rack runners rest on the tub side rails",
    )
    ctx.expect_gap(
        upper_rack,
        lower_rack,
        axis="z",
        min_gap=0.10,
        name="upper rack is clearly above the lower rack",
    )

    # --- open poses: door folds flat forward; racks slide out over it
    with ctx.pose({hinge: math.pi / 2}):
        open_aabb = ctx.part_world_aabb(door)
        ok = open_aabb is not None
        if ok:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ok = (
                oz1 < 0.12  # panel now horizontal and low
                and oy1 > 0.95  # reaches far forward
                and oz0 > 0.0  # never dips below the floor
            )
        ctx.check(
            "fully open door lies horizontal in front of the cabinet, above the floor",
            ok,
            details=f"open door aabb = {open_aabb}",
        )

    rest_pos = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_slide: RACK_TRAVEL, upper_slide: RACK_TRAVEL, hinge: math.pi / 2}):
        ext_pos = ctx.part_world_position(lower_rack)
        ctx.check(
            "lower rack slides forward by 0.45 m",
            rest_pos is not None
            and ext_pos is not None
            and abs((ext_pos[1] - rest_pos[1]) - RACK_TRAVEL) < 1e-6,
            details=f"rest={rest_pos}, extended={ext_pos}",
        )
        for rack, tag in ((upper_rack, "upper"), (lower_rack, "lower")):
            ctx.expect_overlap(
                rack,
                cabinet,
                axes="y",
                min_overlap=0.02,
                name=f"extended {tag} rack stays partly retained in the tub",
            )
            r_aabb = ctx.part_world_aabb(rack)
            ctx.check(
                f"extended {tag} rack projects out of the cabinet front",
                r_aabb is not None and r_aabb[1][1] > BODY_FRONT_Y + 0.30,
                details=f"{tag} rack aabb = {r_aabb}",
            )
            ctx.expect_gap(
                rack,
                door,
                axis="z",
                min_gap=0.0,
                name=f"extended {tag} rack passes above the folded-down door",
            )

    return ctx.report()


object_model = build_object_model()
