from __future__ import annotations

"""Compact countertop air fryer with mechanical rotary dials and a pull-out basket drawer.

Layout convention (body frame):
- +X is the front of the appliance (the drawer slides out along +X).
- +Y is the left side; the object is symmetric in Y.
- +Z is up; the body sits on the floor plane z = 0.

Overall envelope ~0.28 m wide (Y) x 0.32 m deep (X) x 0.33 m tall (Z).
Articulations:
- prismatic basket-drawer slide along +X, travel 0 -> 0.16 m
- revolute timer dial on the top deck (vertical axis, ~300° sweep)
- revolute temperature dial on the top deck (vertical axis, ~270° sweep)
"""

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    KnobTopFeature,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BODY_H = 0.33
BODY_DEPTH_TOP = 0.32  # X extent at the top rim
BODY_WIDTH_TOP = 0.28  # Y extent at the top rim
BODY_DEPTH_BOT = 0.30  # slight inward taper toward the base
BODY_WIDTH_BOT = 0.26
CORNER_R = 0.040

# Drawer pocket cut into the lower front third of the body.
POCKET_X_BACK = -0.100
POCKET_X_FRONT = 0.180  # safely beyond the tapered front wall
POCKET_HALF_W = 0.1125
POCKET_Z0 = 0.018
POCKET_Z1 = 0.158

# Prismatic joint frame: center of the pocket opening on the front face.
JOINT_X = 0.152
JOINT_Z = 0.088
TRAVEL = 0.160

# Drawer face (drawer-local coordinates, origin at the joint frame).
FACE_X0, FACE_X1 = -0.006, 0.008  # body x 0.146 .. 0.160 (slightly proud)
FACE_W = 0.221
FACE_H = 0.136
WINDOW_W = 0.150
WINDOW_H = 0.085

# Cooking basket (drawer-local).
BASKET_LEN = 0.240
BASKET_W = 0.210
BASKET_H = 0.105
BASKET_X1 = -0.004  # front wall embeds 2 mm into the face back
BASKET_CX = BASKET_X1 - BASKET_LEN / 2.0  # -0.124
# Basket rides on internal rails with a small slide clearance above the
# pocket floor (body z 0.018) so the sliding surfaces do not bind.
BASKET_CLEARANCE = 0.004
BASKET_CZ = (POCKET_Z0 - JOINT_Z) + BASKET_CLEARANCE + BASKET_H / 2.0  # -0.0135

# Mechanical rotary dials on the top deck.
DIAL_X = 0.040          # X center of both dials (toward the front of the top)
DIAL_Y_OFFSET = 0.040   # Y offset from center for each dial
DIAL_SHAFT_R = 0.004    # shaft radius
DIAL_SHAFT_H = 0.018    # shaft length protruding above deck
DIAL_MOUNT_Z = BODY_H   # mounting surface at top of body

# Slide rails on the pocket floor that carry the basket. The basket bottom
# (body z 0.022) seats 2 mm into the rail tops (z 0.024): an intentional
# seated-insertion embed that keeps the sliding drawer mechanically carried.
RAIL_LEN = 0.253
RAIL_W = 0.012
RAIL_H = 0.006
RAIL_CX = 0.0285
RAIL_Y = 0.085
RAIL_CZ = POCKET_Z0 + RAIL_H / 2.0  # 0.021


def _rounded_rect_sketch(wp: cq.Workplane, dx: float, dy: float, r: float) -> cq.Workplane:
    return wp.sketch().rect(dx, dy).vertices().fillet(r).finalize()


def _build_body_shell() -> cq.Workplane:
    """Tapered rounded-corner shell with the drawer pocket and top recess cut in."""
    bottom = cq.Sketch().rect(BODY_DEPTH_BOT, BODY_WIDTH_BOT).vertices().fillet(CORNER_R)
    top = cq.Sketch().rect(BODY_DEPTH_TOP, BODY_WIDTH_TOP).vertices().fillet(CORNER_R)
    shell = (
        cq.Workplane("XY")
        .placeSketch(bottom, top.moved(cq.Location(cq.Vector(0.0, 0.0, BODY_H))))
        .loft()
    )

    # Drawer pocket: open rectangular cavity through the lower front wall.
    pocket = (
        cq.Workplane("XY")
        .box(
            POCKET_X_FRONT - POCKET_X_BACK,
            2.0 * POCKET_HALF_W,
            POCKET_Z1 - POCKET_Z0,
        )
        .translate(
            (
                (POCKET_X_FRONT + POCKET_X_BACK) / 2.0,
                0.0,
                (POCKET_Z0 + POCKET_Z1) / 2.0,
            )
        )
    )
    shell = shell.cut(pocket)

    # Shallow recess in the flat top for the dial escutcheon plate.
    recess = (
        cq.Workplane("XY")
        .box(0.160, 0.120, 0.008)
        .translate((0.040, 0.0, BODY_H + 0.004))
    )
    shell = shell.cut(recess)

    # Two shaft holes through the top deck for the rotary dial stems.
    for hole_y in (-DIAL_Y_OFFSET, DIAL_Y_OFFSET):
        hole = (
            cq.Workplane("XY")
            .cylinder(0.020, DIAL_SHAFT_R * 1.4)
            .translate((DIAL_X, hole_y, BODY_H))
        )
        shell = shell.cut(hole)
    return shell


def _build_trim_band() -> cq.Workplane:
    """Copper trim ring wrapping the upper rim; inner profile embeds into the wall."""
    outer = _rounded_rect_sketch(
        cq.Workplane("XY", origin=(0.0, 0.0, 0.316)), 0.330, 0.290, 0.045
    ).extrude(0.014)
    inner = _rounded_rect_sketch(
        cq.Workplane("XY", origin=(0.0, 0.0, 0.300)), 0.302, 0.262, 0.036
    ).extrude(0.040)
    return outer.cut(inner)


def _build_drawer_face() -> cq.Workplane:
    """Rounded drawer front panel with the viewing-window opening cut through."""
    face = (
        cq.Workplane("XY")
        .box(FACE_X1 - FACE_X0, FACE_W, FACE_H)
        .edges("|X")
        .fillet(0.025)
    )
    window = cq.Workplane("XY").box(0.050, WINDOW_W, WINDOW_H)
    face = face.cut(window)
    return face.translate(((FACE_X0 + FACE_X1) / 2.0, 0.0, 0.0))


def _build_handle() -> cq.Workplane:
    """Flat vertical copper handle on a boss, angled slightly forward/down."""
    boss = cq.Workplane("XY").box(0.024, 0.034, 0.014).translate((0.018, 0.0, 0.052))
    bar = (
        cq.Workplane("XY")
        .box(0.008, 0.036, 0.140)
        .rotate((0, 0, 0), (0, 1, 0), -17.0)
        .translate((0.0465, 0.0, -0.0149))
    )
    return boss.union(bar)


def _build_basket() -> cq.Workplane:
    """Open-top hollow cooking basket sliding inside the pocket."""
    outer = cq.Workplane("XY").box(BASKET_LEN, BASKET_W, BASKET_H).edges("|Z").fillet(0.015)
    cavity = (
        cq.Workplane("XY")
        .box(BASKET_LEN - 0.010, BASKET_W - 0.010, 0.120)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, 0.0125))  # leaves a 5 mm floor, opens the top
    )
    return outer.cut(cavity).translate((BASKET_CX, 0.0, BASKET_CZ))


# (center_x, center_y, center_z, yaw_deg, length) in drawer-local coordinates.
_FRIES = [
    # lower layer, embedded into the food bed slab
    (-0.190, -0.040, -0.045, 80.0, 0.090),
    (-0.160, 0.035, -0.045, 10.0, 0.100),
    (-0.100, -0.030, -0.045, -15.0, 0.095),
    (-0.060, 0.045, -0.045, 60.0, 0.080),
    (-0.200, 0.050, -0.045, -30.0, 0.070),
    (-0.070, -0.045, -0.045, 15.0, 0.090),
    (-0.210, 0.000, -0.045, 75.0, 0.080),
    # upper layer, each crossing a lower fry near its center
    (-0.185, -0.035, -0.036, 170.0, 0.085),
    (-0.155, 0.030, -0.036, 100.0, 0.090),
    (-0.100, -0.025, -0.036, 70.0, 0.080),
    (-0.070, -0.040, -0.036, 105.0, 0.080),
]


def _build_fries_heap() -> cq.Workplane:
    """Heap of golden fries: a bed slab plus criss-crossed sticks, one connected solid."""
    heap = cq.Workplane("XY").box(0.200, 0.170, 0.018).translate((-0.124, 0.0, -0.057))
    for fx, fy, fz, yaw, length in _FRIES:
        fry = (
            cq.Workplane("XY")
            .box(length, 0.011, 0.011)
            .rotate((0, 0, 0), (0, 0, 1), yaw)
            .translate((fx, fy, fz))
        )
        heap = heap.union(fry)
    return heap.translate((0.0, 0.0, BASKET_CLEARANCE))


def _build_timer_knob() -> KnobGeometry:
    """Larger skirted timer dial with knurled grip and engraved line indicator."""
    return KnobGeometry(
        0.044,
        0.022,
        body_style="skirted",
        top_diameter=0.036,
        skirt=KnobSkirt(0.054, 0.005, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="knurled", count=48, depth=0.0010, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        top_feature=KnobTopFeature(style="recessed_dot", diameter=0.004, depth=0.0008),
        center=False,
    )


def _build_temp_knob() -> KnobGeometry:
    """Slightly smaller temperature dial with ribbed grip and wedge indicator."""
    return KnobGeometry(
        0.038,
        0.020,
        body_style="skirted",
        top_diameter=0.032,
        skirt=KnobSkirt(0.048, 0.005, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="ribbed", count=24, depth=0.0009, width=0.0014),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )


def _build_dial_shaft() -> cq.Workplane:
    """Short cylindrical shaft that emerges from the deck into the knob bore."""
    return (
        cq.Workplane("XY")
        .cylinder(DIAL_SHAFT_H, DIAL_SHAFT_R)
        .translate((0.0, 0.0, DIAL_SHAFT_H / 2.0))
    )


def _build_escutcheon_plate() -> cq.Workplane:
    """Thin copper plate inset into the top deck recess, surrounding the dial holes."""
    plate = (
        cq.Workplane("XY")
        .box(0.154, 0.114, 0.004)
        .edges("|Z")
        .fillet(0.012)
        .translate((DIAL_X, 0.0, DIAL_MOUNT_Z + 0.002))
    )
    # Cut shaft holes through the plate.
    for hole_y in (-DIAL_Y_OFFSET, DIAL_Y_OFFSET):
        hole = (
            cq.Workplane("XY")
            .cylinder(0.010, DIAL_SHAFT_R * 1.8)
            .translate((DIAL_X, hole_y, DIAL_MOUNT_Z + 0.002))
        )
        plate = plate.cut(hole)
    return plate


def _build_dial_bezel(y_offset: float) -> cq.Workplane:
    """Copper bezel ring around a dial shaft hole on the escutcheon plate."""
    outer = (
        cq.Workplane("XY")
        .cylinder(0.004, 0.030)
        .translate((DIAL_X, y_offset, DIAL_MOUNT_Z + 0.006))
    )
    inner = (
        cq.Workplane("XY")
        .cylinder(0.006, DIAL_SHAFT_R * 1.6)
        .translate((DIAL_X, y_offset, DIAL_MOUNT_Z + 0.005))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_air_fryer")

    gloss_black = model.material("gloss_black", rgba=(0.06, 0.06, 0.065, 1.0))
    copper = model.material("copper", rgba=(0.76, 0.44, 0.30, 1.0))
    tinted_window = model.material("tinted_window", rgba=(0.12, 0.085, 0.06, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.16, 0.16, 0.16, 1.0))
    fries_gold = model.material("fries_gold", rgba=(0.88, 0.62, 0.24, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell"),
        material=gloss_black,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_build_trim_band(), "rim_trim_band"),
        material=copper,
        name="rim_trim_band",
    )
    body.visual(
        mesh_from_cadquery(_build_escutcheon_plate(), "escutcheon_plate"),
        material=copper,
        name="escutcheon_plate",
    )
    for i, y_off in enumerate((-DIAL_Y_OFFSET, DIAL_Y_OFFSET)):
        body.visual(
            mesh_from_cadquery(_build_dial_bezel(y_off), f"dial_bezel_{i}"),
            material=copper,
            name=f"dial_bezel_{i}",
        )
    body.visual(
        Box((0.005, 0.070, 0.016)),
        origin=Origin(xyz=(0.1574, 0.0, 0.210)),
        material=copper,
        name="brand_logo",
    )
    for idx, rail_y in enumerate((RAIL_Y, -RAIL_Y)):
        body.visual(
            Box((RAIL_LEN, RAIL_W, RAIL_H)),
            origin=Origin(xyz=(RAIL_CX, rail_y, RAIL_CZ)),
            material=basket_metal,
            name=f"basket_slide_rail_{idx}",
        )

    # ---------------------------------------------------------------- drawer
    drawer = model.part("basket_drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_face(), "drawer_face"),
        material=gloss_black,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.005, 0.162, 0.097)),
        origin=Origin(xyz=(-0.0035, 0.0, 0.0)),
        material=tinted_window,
        name="window_glass",
    )
    drawer.visual(
        mesh_from_cadquery(_build_handle(), "drawer_handle"),
        material=copper,
        name="handle",
    )
    drawer.visual(
        mesh_from_cadquery(_build_basket(), "cooking_basket"),
        material=basket_metal,
        name="basket",
    )
    drawer.visual(
        mesh_from_cadquery(_build_fries_heap(), "fries_heap"),
        material=fries_gold,
        name="fries_heap",
    )

    # ----------------------------------------------------------------- joint
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(JOINT_X, 0.0, JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.3, lower=0.0, upper=TRAVEL),
    )

    # -------------------------------------------------------- rotary dials
    knob_dark = model.material("knob_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.52, 0.52, 0.54, 1.0))

    # Timer knob (front-right position on the top deck)
    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(_build_timer_knob(), "timer_knob_cap"),
        material=knob_dark,
        name="knob_cap",
    )
    timer_knob.visual(
        mesh_from_cadquery(_build_dial_shaft(), "timer_shaft"),
        material=shaft_steel,
        name="shaft",
    )

    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(DIAL_X, -DIAL_Y_OFFSET, DIAL_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=5.24),
    )

    # Temperature knob (front-left position on the top deck)
    temp_knob = model.part("temp_knob")
    temp_knob.visual(
        mesh_from_geometry(_build_temp_knob(), "temp_knob_cap"),
        material=knob_dark,
        name="knob_cap",
    )
    temp_knob.visual(
        mesh_from_cadquery(_build_dial_shaft(), "temp_shaft"),
        material=shaft_steel,
        name="shaft",
    )

    model.articulation(
        "body_to_temp_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=temp_knob,
        origin=Origin(xyz=(DIAL_X, DIAL_Y_OFFSET, DIAL_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=4.71),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("basket_drawer")
    slide = object_model.get_articulation("drawer_slide")

    # ---- joint plan: prismatic front-back slide, 0 .. 0.16 m
    ctx.check(
        "drawer slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ax = tuple(slide.axis)
    ctx.check(
        "slide axis is the front-back +x axis",
        abs(ax[0] - 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = slide.motion_limits
    ctx.check(
        "slide travel is 0 .. 0.16 m",
        lim is not None and abs(lim.lower) < 1e-9 and abs(lim.upper - 0.16) < 1e-9,
        details=f"limits={lim}",
    )

    # ---- true scale and grounding (closed pose)
    aabb = ctx.part_world_aabb(body)
    ctx.check("body aabb available", aabb is not None)
    if aabb is not None:
        (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
        ctx.check(
            "body depth ~0.32 m",
            0.30 <= (xmax - xmin) <= 0.36,
            details=f"depth={xmax - xmin:.3f}",
        )
        ctx.check(
            "body width ~0.28 m",
            0.26 <= (ymax - ymin) <= 0.31,
            details=f"width={ymax - ymin:.3f}",
        )
        ctx.check(
            "body height ~0.33 m",
            0.31 <= (zmax - zmin) <= 0.345,
            details=f"height={zmax - zmin:.3f}",
        )
        ctx.check("body grounded at z=0", abs(zmin) < 0.002, details=f"zmin={zmin:.4f}")

    # ---- basket seats on the body slide rails (intentional 2 mm embed)
    for rail in ("basket_slide_rail_0", "basket_slide_rail_1"):
        ctx.allow_overlap(
            drawer,
            body,
            elem_a="basket",
            elem_b=rail,
            reason=(
                "Basket bottom seats 2 mm into the slide-rail tops so the "
                "sliding drawer is mechanically carried by the body rails."
            ),
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="z",
            elem_a="basket",
            elem_b=rail,
            min_overlap=0.001,
            name=f"basket seats on {rail}",
        )

    # ---- closed pose: drawer nests flush in the recessed pocket
    ctx.expect_within(
        drawer,
        body,
        axes="y",
        inner_elem="basket",
        margin=0.001,
        name="basket stays centered inside the body pocket",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="basket",
        min_overlap=0.20,
        name="basket is fully inserted when closed",
    )
    face_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_face")
    ctx.check("drawer face aabb available", face_aabb is not None)
    if face_aabb is not None:
        ctx.check(
            "drawer face sits just proud of the front wall",
            0.155 < face_aabb[1][0] < 0.175,
            details=f"face_xmax={face_aabb[1][0]:.4f}",
        )
        ctx.check(
            "drawer face occupies the lower front third",
            face_aabb[0][2] > 0.010 and face_aabb[1][2] < 0.170,
            details=f"face_z=({face_aabb[0][2]:.3f},{face_aabb[1][2]:.3f})",
        )
    glass_aabb = ctx.part_element_world_aabb(drawer, elem="window_glass")
    if face_aabb is not None and glass_aabb is not None:
        ctx.check(
            "viewing window is recessed behind the face front",
            glass_aabb[1][0] < face_aabb[1][0] - 0.005,
            details=f"glass_xmax={glass_aabb[1][0]:.4f}",
        )
    ctx.expect_within(
        drawer,
        drawer,
        axes="yz",
        inner_elem="window_glass",
        outer_elem="drawer_face",
        name="window pane framed by the drawer face",
    )

    # ---- handle: copper bar proud of the face, angled downward
    handle_aabb = ctx.part_element_world_aabb(drawer, elem="handle")
    ctx.check("handle aabb available", handle_aabb is not None)
    if handle_aabb is not None and face_aabb is not None:
        ctx.check(
            "handle stands proud of the drawer face",
            handle_aabb[1][0] > face_aabb[1][0] + 0.03,
            details=f"handle_xmax={handle_aabb[1][0]:.4f}",
        )
        ctx.check(
            "handle bar angles downward toward the counter",
            handle_aabb[0][2] < 0.020 and handle_aabb[1][2] > 0.130,
            details=f"handle_z=({handle_aabb[0][2]:.4f},{handle_aabb[1][2]:.4f})",
        )
        ctx.check(
            "handle stays above the floor",
            handle_aabb[0][2] > 0.0,
            details=f"handle_zmin={handle_aabb[0][2]:.4f}",
        )

    # ---- food heap sits inside the open-top basket
    ctx.expect_within(
        drawer,
        drawer,
        axes="xy",
        inner_elem="fries_heap",
        outer_elem="basket",
        name="fries heap contained by the basket walls",
    )
    basket_aabb = ctx.part_element_world_aabb(drawer, elem="basket")
    fries_aabb = ctx.part_element_world_aabb(drawer, elem="fries_heap")
    if basket_aabb is not None and fries_aabb is not None:
        ctx.check(
            "basket is open-topped above the food",
            fries_aabb[1][2] < basket_aabb[1][2],
            details=f"fries_zmax={fries_aabb[1][2]:.4f}, basket_zmax={basket_aabb[1][2]:.4f}",
        )

    # ---- escutcheon plate and copper dial bezels on the top deck
    escutcheon = ctx.part_element_world_aabb(body, elem="escutcheon_plate")
    if escutcheon is not None:
        ctx.check(
            "escutcheon plate sits on the flat top deck",
            0.325 < escutcheon[0][2] < 0.335 and escutcheon[1][2] < 0.340,
            details=f"escutcheon_z=({escutcheon[0][2]:.4f},{escutcheon[1][2]:.4f})",
        )
    band = ctx.part_element_world_aabb(body, elem="rim_trim_band")
    if band is not None:
        ctx.check(
            "copper trim band wraps the upper rim",
            band[1][2] > 0.325 and band[1][0] > 0.162 and band[1][1] > 0.142,
            details=f"band_max={band[1]}",
        )
    logo = ctx.part_element_world_aabb(body, elem="brand_logo")
    if logo is not None:
        ctx.check(
            "brand logo on the upper front face",
            logo[1][0] > 0.155 and 0.18 < logo[0][2] < 0.24,
            details=f"logo={logo}",
        )

    # ---- rotary dials: revolute joints on the top deck
    timer_joint = object_model.get_articulation("body_to_timer_knob")
    temp_joint = object_model.get_articulation("body_to_temp_knob")
    timer_part = object_model.get_part("timer_knob")
    temp_part = object_model.get_part("temp_knob")

    ctx.check(
        "timer dial is revolute",
        timer_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={timer_joint.articulation_type}",
    )
    ctx.check(
        "temp dial is revolute",
        temp_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={temp_joint.articulation_type}",
    )

    # Both dials rotate around vertical axis on the top deck.
    for jname, j in (("timer", timer_joint), ("temp", temp_joint)):
        ax = tuple(j.axis)
        ctx.check(
            f"{jname} dial axis is vertical (z)",
            abs(ax[2] - 1.0) < 1e-9 and abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9,
            details=f"axis={ax}",
        )
        lim = j.motion_limits
        ctx.check(
            f"{jname} dial has realistic sweep",
            lim is not None and lim.upper > 3.0 and lim.upper < 6.3,
            details=f"limits={lim}",
        )

    # Dial knobs sit on the top deck surface, not floating.
    for kname, kpart in (("timer_knob", timer_part), ("temp_knob", temp_part)):
        knob_aabb = ctx.part_element_world_aabb(kpart, elem="knob_cap")
        if knob_aabb is not None:
            ctx.check(
                f"{kname} cap sits on the top deck",
                0.325 < knob_aabb[0][2] < 0.340 and knob_aabb[1][2] < 0.365,
                details=f"knob_z=({knob_aabb[0][2]:.4f},{knob_aabb[1][2]:.4f})",
            )
        shaft_aabb = ctx.part_element_world_aabb(kpart, elem="shaft")
        if shaft_aabb is not None:
            ctx.check(
                f"{kname} shaft emerges from the deck",
                shaft_aabb[0][2] > 0.325 and shaft_aabb[1][2] < 0.355,
                details=f"shaft_z=({shaft_aabb[0][2]:.4f},{shaft_aabb[1][2]:.4f})",
            )

    # Rotating the timer dial changes its indicator orientation.
    rest_pos = ctx.part_world_position(timer_part)
    with ctx.pose({timer_joint: 2.5}):
        turned_pos = ctx.part_world_position(timer_part)
        # The part origin stays at the same location (pure rotation around Z).
        if rest_pos is not None and turned_pos is not None:
            ctx.check(
                "timer dial rotates in place without translation",
                abs(turned_pos[0] - rest_pos[0]) < 1e-6
                and abs(turned_pos[1] - rest_pos[1]) < 1e-6
                and abs(turned_pos[2] - rest_pos[2]) < 1e-6,
                details=f"rest={rest_pos}, turned={turned_pos}",
            )

    # Knob shafts embed into the body deck (intentional shaft-in-hole seating).
    for kpart_name, shaft_elem in (("timer_knob", "shaft"), ("temp_knob", "shaft")):
        ctx.allow_overlap(
            kpart_name,
            "body",
            elem_a=shaft_elem,
            reason="Dial shaft is intentionally seated through the deck shaft hole.",
        )

    # ---- open pose: drawer translates +0.16 m forward, basket exposed but retained
    closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: TRAVEL}):
        open_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="basket",
            min_overlap=0.05,
            name="basket retains insertion at full travel",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="y",
            inner_elem="basket",
            margin=0.001,
            name="extended basket stays centered in the pocket",
        )
        basket_open = ctx.part_element_world_aabb(drawer, elem="basket")
        if basket_open is not None:
            ctx.check(
                "open basket is exposed beyond the front wall",
                basket_open[1][0] > 0.25,
                details=f"basket_xmax_open={basket_open[1][0]:.4f}",
            )
    ctx.check(
        "drawer slides forward by the full travel",
        closed_pos is not None
        and open_pos is not None
        and abs((open_pos[0] - closed_pos[0]) - TRAVEL) < 1e-6
        and abs(open_pos[1] - closed_pos[1]) < 1e-9
        and abs(open_pos[2] - closed_pos[2]) < 1e-9,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
