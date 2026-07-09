from __future__ import annotations

"""Cylindrical drum air fryer with a pull-out basket drawer.

Layout convention (body frame):
- +X is the front of the appliance (the drawer slides out along +X).
- +Y is the left side; the object is symmetric in Y.
- +Z is up; the body sits on the floor plane z = 0.

The main body is a cylindrical drum (circular footprint, built by revolving
a rectangular profile around the Z axis) with a flat front facet where the
basket drawer pulls out. Body diameter ~0.32 m, height 0.33 m.
The single articulation is the prismatic basket-drawer slide along +X,
travel 0 -> 0.16 m from flush-closed to fully extended.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
BODY_RADIUS = 0.160       # drum radius (diameter 0.32 m)
BODY_HEIGHT = 0.330       # total height
FLAT_FRONT_X = 0.110      # flat front facet distance from cylinder axis
CHORD_HALF = math.sqrt(BODY_RADIUS**2 - FLAT_FRONT_X**2)  # ~0.116

# Drawer pocket cut into the lower front of the drum
POCKET_X_BACK = -0.095
POCKET_X_FRONT = 0.115    # slightly past flat front for through-cut
POCKET_HALF_W = 0.108     # pocket half-width 0.216 m
POCKET_Z0 = 0.020
POCKET_Z1 = 0.155

# Prismatic joint frame: center of the pocket opening on the flat front face
JOINT_X = FLAT_FRONT_X    # 0.110
JOINT_Z = (POCKET_Z0 + POCKET_Z1) / 2.0  # 0.0875
TRAVEL = 0.160

# Drawer face (drawer-local coordinates, origin at joint frame)
FACE_X0, FACE_X1 = -0.006, 0.008
FACE_W = 2.0 * POCKET_HALF_W - 0.004    # 0.212
FACE_H = POCKET_Z1 - POCKET_Z0 - 0.004  # 0.131
WINDOW_W = 0.150
WINDOW_H = 0.085

# Cooking basket (drawer-local)
BASKET_LEN = 0.190
BASKET_W = 0.200
BASKET_H = 0.105
BASKET_X1 = -0.004       # front wall embeds into the face back
BASKET_CX = BASKET_X1 - BASKET_LEN / 2.0  # -0.099
BASKET_CLEARANCE = 0.004
BASKET_CZ = (POCKET_Z0 - JOINT_Z) + BASKET_CLEARANCE + BASKET_H / 2.0

# Top glass panel
TOP_GLASS_Z = BODY_HEIGHT - 0.009  # 0.321; glass bottom at recess floor (0.318)

# Slide rails on the pocket floor that carry the basket
RAIL_LEN = 0.200
RAIL_W = 0.012
RAIL_H = 0.006
RAIL_CX = (POCKET_X_BACK + POCKET_X_FRONT) / 2.0  # 0.010 world X
RAIL_Y = 0.082
RAIL_CZ = POCKET_Z0 + RAIL_H / 2.0  # 0.023


# =========================================================== geometry helpers


def _build_drum_shell() -> cq.Workplane:
    """Cylindrical drum shell with flat front facet, drawer pocket, and top recess.

    The main cylinder is built by revolving a rectangular profile around the
    Z axis (lathe/revolve operation), then boolean-cutting the flat front facet,
    drawer pocket cavity, and circular top-panel recess.
    """
    # --- revolve a rectangular profile around Z to form the cylinder
    drum = (
        cq.Workplane("XZ")
        .moveTo(0.001, 0)
        .lineTo(BODY_RADIUS, 0)
        .lineTo(BODY_RADIUS, BODY_HEIGHT)
        .lineTo(0.001, BODY_HEIGHT)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )

    # --- flat front facet: remove everything beyond FLAT_FRONT_X
    cut_w = BODY_RADIUS + 0.010
    flat_cut = (
        cq.Workplane("XY")
        .box(cut_w, 2.0 * BODY_RADIUS + 0.020, BODY_HEIGHT + 0.020)
        .translate((FLAT_FRONT_X + cut_w / 2.0, 0, BODY_HEIGHT / 2.0))
    )
    drum = drum.cut(flat_cut)

    # --- drawer pocket: rectangular cavity through the flat front
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
                0,
                (POCKET_Z0 + POCKET_Z1) / 2.0,
            )
        )
    )
    drum = drum.cut(pocket)

    # --- shallow circular recess in the flat top for the glass touch panel
    recess = (
        cq.Workplane("XY", origin=(0, 0, BODY_HEIGHT - 0.012))
        .circle(BODY_RADIUS - 0.018)
        .extrude(0.015)
    )
    drum = drum.cut(recess)

    return drum


def _build_trim_band() -> cq.Workplane:
    """Copper trim ring wrapping the upper rim of the cylindrical drum.

    D-shaped ring: circular on the sides and back, flat-cut on the front
    to match the body contour.
    """
    outer = (
        cq.Workplane("XY", origin=(0, 0, 0.316))
        .circle(BODY_RADIUS + 0.004)
        .extrude(0.014)
    )
    inner = (
        cq.Workplane("XY", origin=(0, 0, 0.300))
        .circle(BODY_RADIUS - 0.008)
        .extrude(0.040)
    )
    band = outer.cut(inner)
    # match the flat front facet
    cut_w = BODY_RADIUS + 0.005
    flat_cut = (
        cq.Workplane("XY")
        .box(cut_w, 2.0 * BODY_RADIUS + 0.020, 0.050)
        .translate((FLAT_FRONT_X + cut_w / 2.0, 0, 0.325))
    )
    band = band.cut(flat_cut)
    return band


def _build_top_glass() -> cq.Workplane:
    """Circular glass touch-control panel disk, D-trimmed to match the flat front."""
    glass = (
        cq.Workplane("XY", origin=(0, 0, TOP_GLASS_Z - 0.003))
        .circle(BODY_RADIUS - 0.022)
        .extrude(0.006)
    )
    # cut flat front to match body D-shape
    cut_w = BODY_RADIUS
    flat_cut = (
        cq.Workplane("XY")
        .box(cut_w, 2.0 * BODY_RADIUS + 0.020, 0.020)
        .translate((FLAT_FRONT_X + cut_w / 2.0, 0, TOP_GLASS_Z))
    )
    glass = glass.cut(flat_cut)
    return glass


def _build_drawer_face() -> cq.Workplane:
    """Rounded drawer front panel with the viewing-window opening cut through."""
    face = (
        cq.Workplane("XY")
        .box(FACE_X1 - FACE_X0, FACE_W, FACE_H)
        .edges("|X")
        .fillet(0.022)
    )
    window = cq.Workplane("XY").box(0.050, WINDOW_W, WINDOW_H)
    face = face.cut(window)
    return face.translate(((FACE_X0 + FACE_X1) / 2.0, 0, 0))


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
    outer = (
        cq.Workplane("XY")
        .box(BASKET_LEN, BASKET_W, BASKET_H)
        .edges("|Z")
        .fillet(0.012)
    )
    cavity = (
        cq.Workplane("XY")
        .box(BASKET_LEN - 0.010, BASKET_W - 0.010, 0.120)
        .edges("|Z")
        .fillet(0.010)
        .translate((0, 0, 0.0125))
    )
    return outer.cut(cavity).translate((BASKET_CX, 0, BASKET_CZ))


# Fries stick positions in drawer-local coordinates:
# (center_x, center_y, center_z, yaw_deg, length)
_FRIES = [
    # lower layer embedded into the food bed slab
    (-0.155, -0.040, -0.045, 80.0, 0.080),
    (-0.130, 0.035, -0.045, 10.0, 0.085),
    (-0.080, -0.030, -0.045, -15.0, 0.080),
    (-0.055, 0.045, -0.045, 60.0, 0.070),
    (-0.160, 0.050, -0.045, -30.0, 0.065),
    (-0.060, -0.045, -0.045, 15.0, 0.080),
    (-0.170, 0.000, -0.045, 75.0, 0.070),
    # upper layer crossing lower fries
    (-0.150, -0.035, -0.036, 170.0, 0.075),
    (-0.125, 0.030, -0.036, 100.0, 0.080),
    (-0.080, -0.025, -0.036, 70.0, 0.070),
    (-0.060, -0.040, -0.036, 105.0, 0.070),
]


def _build_fries_heap() -> cq.Workplane:
    """Heap of golden fries: a bed slab plus criss-crossed sticks, one connected solid."""
    heap = (
        cq.Workplane("XY")
        .box(0.160, 0.160, 0.018)
        .translate((BASKET_CX, 0, -0.057))
    )
    for fx, fy, fz, yaw, length in _FRIES:
        fry = (
            cq.Workplane("XY")
            .box(length, 0.011, 0.011)
            .rotate((0, 0, 0), (0, 0, 1), yaw)
            .translate((fx, fy, fz))
        )
        heap = heap.union(fry)
    return heap.translate((0, 0, BASKET_CLEARANCE))


# =========================================================== object model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_drum_air_fryer")

    gloss_black = model.material("gloss_black", rgba=(0.06, 0.06, 0.065, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.03, 0.03, 0.035, 1.0))
    copper = model.material("copper", rgba=(0.76, 0.44, 0.30, 1.0))
    tinted_window = model.material("tinted_window", rgba=(0.12, 0.085, 0.06, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.16, 0.16, 0.16, 1.0))
    fries_gold = model.material("fries_gold", rgba=(0.88, 0.62, 0.24, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_drum_shell(), "drum_shell"),
        material=gloss_black,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_build_trim_band(), "rim_trim_band"),
        material=copper,
        name="rim_trim_band",
    )
    body.visual(
        mesh_from_cadquery(_build_top_glass(), "top_glass_disk"),
        material=smoked_glass,
        name="top_glass_panel",
    )
    body.visual(
        Box((0.004, 0.070, 0.016)),
        origin=Origin(xyz=(FLAT_FRONT_X + 0.002, 0.0, 0.210)),
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
        Box((0.005, WINDOW_W + 0.012, WINDOW_H + 0.012)),
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

    return model


# =========================================================== tests


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

    # ---- cylindrical body: circular footprint, flat front facet
    aabb = ctx.part_world_aabb(body)
    ctx.check("body aabb available", aabb is not None)
    if aabb is not None:
        (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
        body_depth = xmax - xmin
        body_width = ymax - ymin
        body_height = zmax - zmin
        ctx.check(
            "body height ~0.33 m",
            0.31 <= body_height <= 0.345,
            details=f"height={body_height:.3f}",
        )
        ctx.check(
            "body width (diameter) ~0.32 m",
            0.28 <= body_width <= 0.36,
            details=f"width={body_width:.3f}",
        )
        ctx.check(
            "cylindrical footprint: width similar to depth",
            abs(body_width - body_depth) < 0.08,
            details=f"width={body_width:.3f}, depth={body_depth:.3f}",
        )
        ctx.check(
            "body grounded at z=0",
            abs(zmin) < 0.002,
            details=f"zmin={zmin:.4f}",
        )
        ctx.check(
            "flat front facet truncates the drum cylinder",
            abs(xmax - FLAT_FRONT_X) < 0.015,
            details=f"xmax={xmax:.4f}, flat_front={FLAT_FRONT_X}",
        )
        ctx.check(
            "drum back extends to approximately -R",
            abs(xmin + BODY_RADIUS) < 0.015,
            details=f"xmin={xmin:.4f}",
        )

    # ---- basket seats on the body slide rails (intentional seated embed)
    for rail in ("basket_slide_rail_0", "basket_slide_rail_1"):
        ctx.allow_overlap(
            drawer,
            body,
            elem_a="basket",
            elem_b=rail,
            reason=(
                "Basket bottom seats into the slide-rail tops so the "
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
        min_overlap=0.15,
        name="basket is fully inserted when closed",
    )
    face_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_face")
    ctx.check("drawer face aabb available", face_aabb is not None)
    if face_aabb is not None:
        ctx.check(
            "drawer face sits at the flat front facet",
            FLAT_FRONT_X - 0.010 < face_aabb[1][0] < FLAT_FRONT_X + 0.020,
            details=f"face_xmax={face_aabb[1][0]:.4f}",
        )
        ctx.check(
            "drawer face occupies the lower front section",
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
            handle_aabb[0][2] < 0.030 and handle_aabb[1][2] > 0.120,
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

    # ---- top glass panel and copper trim band on cylindrical drum
    top_glass = ctx.part_element_world_aabb(body, elem="top_glass_panel")
    if top_glass is not None:
        ctx.check(
            "glass touch panel caps the flat drum top",
            0.315 < top_glass[1][2] < 0.335,
            details=f"glass_zmax={top_glass[1][2]:.4f}",
        )
    band = ctx.part_element_world_aabb(body, elem="rim_trim_band")
    if band is not None:
        ctx.check(
            "copper trim band wraps the upper rim",
            band[1][2] > 0.310 and band[0][2] > 0.300,
            details=f"band_z=({band[0][2]:.3f},{band[1][2]:.3f})",
        )
        ctx.check(
            "trim band spans the drum diameter in Y",
            band[1][1] > BODY_RADIUS - 0.010,
            details=f"band_ymax={band[1][1]:.4f}",
        )
    logo = ctx.part_element_world_aabb(body, elem="brand_logo")
    if logo is not None:
        ctx.check(
            "brand logo on the flat front face above the pocket",
            logo[1][0] > FLAT_FRONT_X - 0.005 and 0.18 < logo[0][2] < 0.26,
            details=f"logo={logo}",
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
            min_overlap=0.03,
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
                basket_open[1][0] > 0.22,
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
