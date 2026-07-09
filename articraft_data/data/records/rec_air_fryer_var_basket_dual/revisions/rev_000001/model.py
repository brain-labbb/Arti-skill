from __future__ import annotations

"""Dual-basket countertop air fryer with two independent pull-out drawers.

Layout convention (body frame):
- +X is the front of the appliance (drawers slide out along +X).
- +Y is the left side; the object is symmetric in Y.
- +Z is up; the body sits on the floor plane z = 0.

Overall envelope ~0.40 m wide (Y) x 0.32 m deep (X) x 0.33 m tall (Z).
Two independent prismatic basket-drawer slides along +X,
travel 0 -> 0.16 m from flush-closed to fully extended.
"""

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
BODY_H = 0.33
BODY_DEPTH_TOP = 0.32
BODY_WIDTH_TOP = 0.40        # widened for dual side-by-side baskets
BODY_DEPTH_BOT = 0.30
BODY_WIDTH_BOT = 0.38
CORNER_R = 0.040

# Two pocket openings, symmetric about Y=0 with a center divider wall.
DIVIDER_HALF = 0.012         # half-thickness of the center divider
POCKET_HW = 0.078            # half-width of each pocket opening (face fits within)
POCKET_CY = (DIVIDER_HALF + POCKET_HW, -(DIVIDER_HALF + POCKET_HW))  # ±0.090
POCKET_X_BACK = -0.100
POCKET_X_FRONT = 0.180
POCKET_Z0 = 0.018
POCKET_Z1 = 0.158

# Prismatic joint frame: center of each pocket opening on the front face.
JOINT_X = 0.152
JOINT_Z = 0.088
TRAVEL = 0.160

# Drawer face (drawer-local coordinates, origin at the joint frame).
FACE_X0, FACE_X1 = -0.006, 0.008
FACE_W = 0.152
FACE_H = 0.136
WINDOW_W = 0.100
WINDOW_H = 0.085

# Cooking basket (drawer-local).
BASKET_LEN = 0.240
BASKET_W = 0.138
BASKET_H = 0.105
BASKET_X1 = -0.004
BASKET_CX = BASKET_X1 - BASKET_LEN / 2.0
BASKET_CLEARANCE = 0.004
BASKET_CZ = (POCKET_Z0 - JOINT_Z) + BASKET_CLEARANCE + BASKET_H / 2.0

TOP_GLASS_Z = 0.3265

# Slide rails: two per drawer, mounted on the pocket floor.
RAIL_LEN = 0.253
RAIL_W = 0.012
RAIL_H = 0.006
RAIL_CX = 0.0285
RAIL_OFFSET = 0.050          # Y offset from pocket center to each rail
RAIL_CZ = POCKET_Z0 + RAIL_H / 2.0

NUM_BASKETS = 2


# -------------------------------------------------------- geometry helpers
def _rounded_rect_sketch(wp: cq.Workplane, dx: float, dy: float, r: float) -> cq.Workplane:
    return wp.sketch().rect(dx, dy).vertices().fillet(r).finalize()


def _build_body_shell() -> cq.Workplane:
    """Tapered rounded-corner shell with two drawer pockets and top recess."""
    bottom = cq.Sketch().rect(BODY_DEPTH_BOT, BODY_WIDTH_BOT).vertices().fillet(CORNER_R)
    top = cq.Sketch().rect(BODY_DEPTH_TOP, BODY_WIDTH_TOP).vertices().fillet(CORNER_R)
    shell = (
        cq.Workplane("XY")
        .placeSketch(bottom, top.moved(cq.Location(cq.Vector(0.0, 0.0, BODY_H))))
        .loft()
    )

    # Two drawer pocket openings cut through the lower front wall.
    for cy in POCKET_CY:
        pocket = (
            cq.Workplane("XY")
            .box(
                POCKET_X_FRONT - POCKET_X_BACK,
                2.0 * POCKET_HW,
                POCKET_Z1 - POCKET_Z0,
            )
            .translate(
                (
                    (POCKET_X_FRONT + POCKET_X_BACK) / 2.0,
                    cy,
                    (POCKET_Z0 + POCKET_Z1) / 2.0,
                )
            )
        )
        shell = shell.cut(pocket)

    # Shallow recess in the flat top for the glass touch panel.
    recess = (
        cq.Workplane("XY")
        .box(0.240, 0.330, 0.015)
        .translate((0.0, 0.0, BODY_H + 0.0025))
    )
    shell = shell.cut(recess)
    return shell


def _build_trim_band() -> cq.Workplane:
    """Copper trim ring wrapping the upper rim; inner profile embeds into the wall."""
    outer = _rounded_rect_sketch(
        cq.Workplane("XY", origin=(0.0, 0.0, 0.316)), 0.330, 0.410, 0.045
    ).extrude(0.014)
    inner = _rounded_rect_sketch(
        cq.Workplane("XY", origin=(0.0, 0.0, 0.300)), 0.302, 0.382, 0.036
    ).extrude(0.040)
    return outer.cut(inner)


def _build_drawer_face() -> cq.Workplane:
    """Rounded drawer front panel with the viewing-window opening cut through."""
    face = (
        cq.Workplane("XY")
        .box(FACE_X1 - FACE_X0, FACE_W, FACE_H)
        .edges("|X")
        .fillet(0.020)
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
        .translate((0.0, 0.0, 0.0125))
    )
    return outer.cut(cavity).translate((BASKET_CX, 0.0, BASKET_CZ))


# Fry positions scaled for the narrower dual basket (Y range ±0.025).
_FRIES = [
    (-0.190, -0.020, -0.045, 80.0, 0.070),
    (-0.160, 0.015, -0.045, 10.0, 0.080),
    (-0.100, -0.014, -0.045, -15.0, 0.075),
    (-0.060, 0.022, -0.045, 60.0, 0.065),
    (-0.200, 0.024, -0.045, -30.0, 0.060),
    (-0.070, -0.020, -0.045, 15.0, 0.075),
    (-0.210, 0.000, -0.045, 75.0, 0.065),
    (-0.185, -0.018, -0.036, 170.0, 0.070),
    (-0.155, 0.014, -0.036, 100.0, 0.075),
    (-0.100, -0.012, -0.036, 70.0, 0.065),
    (-0.070, -0.020, -0.036, 105.0, 0.065),
]


def _build_fries_heap() -> cq.Workplane:
    """Heap of golden fries: a bed slab plus criss-crossed sticks, one connected solid."""
    heap = cq.Workplane("XY").box(0.195, 0.090, 0.018).translate((-0.124, 0.0, -0.057))
    for fx, fy, fz, yaw, length in _FRIES:
        fry = (
            cq.Workplane("XY")
            .box(length, 0.011, 0.011)
            .rotate((0, 0, 0), (0, 0, 1), yaw)
            .translate((fx, fy, fz))
        )
        heap = heap.union(fry)
    return heap.translate((0.0, 0.0, BASKET_CLEARANCE))


def _populate_drawer(
    part,
    face_mesh,
    handle_mesh,
    basket_mesh,
    fries_mesh,
    *,
    mat_black,
    mat_window,
    mat_copper,
    mat_basket,
    mat_fries,
):
    """Shared basket+drawer geometry helper: adds all visuals for one drawer."""
    part.visual(face_mesh, material=mat_black, name="drawer_face")
    part.visual(
        Box((0.005, WINDOW_W + 0.012, WINDOW_H + 0.012)),
        origin=Origin(xyz=(-0.0035, 0.0, 0.0)),
        material=mat_window,
        name="window_glass",
    )
    part.visual(handle_mesh, material=mat_copper, name="handle")
    part.visual(basket_mesh, material=mat_basket, name="basket")
    part.visual(fries_mesh, material=mat_fries, name="fries_heap")


# ---------------------------------------------------------- model builder
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_basket_air_fryer")

    gloss_black = model.material("gloss_black", rgba=(0.06, 0.06, 0.065, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.03, 0.03, 0.035, 1.0))
    copper = model.material("copper", rgba=(0.76, 0.44, 0.30, 1.0))
    tinted_window = model.material("tinted_window", rgba=(0.12, 0.085, 0.06, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.16, 0.16, 0.16, 1.0))
    fries_gold = model.material("fries_gold", rgba=(0.88, 0.62, 0.24, 1.0))

    # ---------------------------------------------------------------- body
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
        Box((0.238, 0.320, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, TOP_GLASS_Z)),
        material=smoked_glass,
        name="top_glass_panel",
    )
    body.visual(
        Box((0.005, 0.070, 0.016)),
        origin=Origin(xyz=(0.1574, 0.0, 0.210)),
        material=copper,
        name="brand_logo",
    )

    # Slide rails: 2 per drawer, 4 total on the body.
    for i in range(NUM_BASKETS):
        cy = POCKET_CY[i]
        for j, dy in enumerate((RAIL_OFFSET, -RAIL_OFFSET)):
            body.visual(
                Box((RAIL_LEN, RAIL_W, RAIL_H)),
                origin=Origin(xyz=(RAIL_CX, cy + dy, RAIL_CZ)),
                material=basket_metal,
                name=f"slide_rail_{i}_{j}",
            )

    # Pre-build shared drawer meshes (identical geometry for both baskets).
    face_mesh = mesh_from_cadquery(_build_drawer_face(), "drawer_face")
    handle_mesh = mesh_from_cadquery(_build_handle(), "drawer_handle")
    basket_mesh = mesh_from_cadquery(_build_basket(), "cooking_basket")
    fries_mesh = mesh_from_cadquery(_build_fries_heap(), "fries_heap")

    # ------------------------------------------ dual drawers (for loop)
    for i in range(NUM_BASKETS):
        cy = POCKET_CY[i]
        drawer = model.part(f"drawer_{i}")

        _populate_drawer(
            drawer,
            face_mesh,
            handle_mesh,
            basket_mesh,
            fries_mesh,
            mat_black=gloss_black,
            mat_window=tinted_window,
            mat_copper=copper,
            mat_basket=basket_metal,
            mat_fries=fries_gold,
        )

        model.articulation(
            f"drawer_slide_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(JOINT_X, cy, JOINT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=0.3, lower=0.0, upper=TRAVEL
            ),
        )

    return model


# ----------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawers = [object_model.get_part(f"drawer_{i}") for i in range(NUM_BASKETS)]
    slides = [
        object_model.get_articulation(f"drawer_slide_{i}") for i in range(NUM_BASKETS)
    ]

    # ---- joint plan: 2 independent prismatic slides, +X, 0 .. 0.16 m
    for i in range(NUM_BASKETS):
        slide = slides[i]
        ctx.check(
            f"drawer_{i} slide is prismatic",
            slide.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={slide.articulation_type}",
        )
        ax = tuple(slide.axis)
        ctx.check(
            f"drawer_{i} slide axis is +x",
            abs(ax[0] - 1.0) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2]) < 1e-9,
            details=f"axis={ax}",
        )
        lim = slide.motion_limits
        ctx.check(
            f"drawer_{i} slide travel 0..0.16 m",
            lim is not None and abs(lim.lower) < 1e-9 and abs(lim.upper - 0.16) < 1e-9,
            details=f"limits={lim}",
        )

    # ---- true scale and grounding (body widened for dual basket)
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
            "body width ~0.40 m (dual basket)",
            0.38 <= (ymax - ymin) <= 0.43,
            details=f"width={ymax - ymin:.3f}",
        )
        ctx.check(
            "body height ~0.33 m",
            0.31 <= (zmax - zmin) <= 0.345,
            details=f"height={zmax - zmin:.3f}",
        )
        ctx.check(
            "body grounded at z=0",
            abs(zmin) < 0.002,
            details=f"zmin={zmin:.4f}",
        )

    # ---- per-drawer closed-pose checks
    for i in range(NUM_BASKETS):
        drawer = drawers[i]

        # Rail seating: basket embeds 2 mm into rail tops (intentional).
        for j in range(2):
            rail_name = f"slide_rail_{i}_{j}"
            ctx.allow_overlap(
                drawer,
                body,
                elem_a="basket",
                elem_b=rail_name,
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
                elem_b=rail_name,
                min_overlap=0.001,
                name=f"drawer_{i} basket seats on {rail_name}",
            )

        # Basket centered in pocket (Y) and fully inserted (X).
        ctx.expect_within(
            drawer,
            body,
            axes="y",
            inner_elem="basket",
            margin=0.001,
            name=f"drawer_{i} basket centered in pocket",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="basket",
            min_overlap=0.20,
            name=f"drawer_{i} basket fully inserted when closed",
        )

        # Drawer face proud of front wall, in the lower third.
        face_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_face")
        ctx.check(f"drawer_{i} face aabb available", face_aabb is not None)
        if face_aabb is not None:
            ctx.check(
                f"drawer_{i} face proud of front wall",
                0.155 < face_aabb[1][0] < 0.175,
                details=f"face_xmax={face_aabb[1][0]:.4f}",
            )
            ctx.check(
                f"drawer_{i} face occupies lower front third",
                face_aabb[0][2] > 0.010 and face_aabb[1][2] < 0.170,
                details=f"face_z=({face_aabb[0][2]:.3f},{face_aabb[1][2]:.3f})",
            )

        # Window recessed behind the face and framed by it.
        glass_aabb = ctx.part_element_world_aabb(drawer, elem="window_glass")
        if face_aabb is not None and glass_aabb is not None:
            ctx.check(
                f"drawer_{i} window recessed behind face",
                glass_aabb[1][0] < face_aabb[1][0] - 0.005,
                details=f"glass_xmax={glass_aabb[1][0]:.4f}",
            )
        ctx.expect_within(
            drawer,
            drawer,
            axes="yz",
            inner_elem="window_glass",
            outer_elem="drawer_face",
            name=f"drawer_{i} window framed by face",
        )

        # Handle proud of face, above floor.
        handle_aabb = ctx.part_element_world_aabb(drawer, elem="handle")
        if handle_aabb is not None and face_aabb is not None:
            ctx.check(
                f"drawer_{i} handle proud of face",
                handle_aabb[1][0] > face_aabb[1][0] + 0.03,
                details=f"handle_xmax={handle_aabb[1][0]:.4f}",
            )
            ctx.check(
                f"drawer_{i} handle stays above floor",
                handle_aabb[0][2] > 0.0,
                details=f"handle_zmin={handle_aabb[0][2]:.4f}",
            )

        # Fries contained in the open-top basket.
        ctx.expect_within(
            drawer,
            drawer,
            axes="xy",
            inner_elem="fries_heap",
            outer_elem="basket",
            name=f"drawer_{i} fries contained in basket",
        )
        basket_aabb = ctx.part_element_world_aabb(drawer, elem="basket")
        fries_aabb = ctx.part_element_world_aabb(drawer, elem="fries_heap")
        if basket_aabb is not None and fries_aabb is not None:
            ctx.check(
                f"drawer_{i} basket open-topped above food",
                fries_aabb[1][2] < basket_aabb[1][2],
                details=(
                    f"fries_zmax={fries_aabb[1][2]:.4f}, "
                    f"basket_zmax={basket_aabb[1][2]:.4f}"
                ),
            )

    # ---- dual-basket layout: drawers side by side in Y
    d0_pos = ctx.part_world_position(drawers[0])
    d1_pos = ctx.part_world_position(drawers[1])
    ctx.check(
        "drawers are side by side (separated in Y)",
        d0_pos is not None
        and d1_pos is not None
        and abs(d0_pos[1] - d1_pos[1]) > 0.10,
        details=f"d0_y={d0_pos[1] if d0_pos else None}, d1_y={d1_pos[1] if d1_pos else None}",
    )

    # ---- top glass panel and copper trim band
    top_glass = ctx.part_element_world_aabb(body, elem="top_glass_panel")
    if top_glass is not None:
        ctx.check(
            "glass touch panel caps the flat top",
            0.320 < top_glass[1][2] < 0.335,
            details=f"glass_zmax={top_glass[1][2]:.4f}",
        )
    band = ctx.part_element_world_aabb(body, elem="rim_trim_band")
    if band is not None:
        ctx.check(
            "copper trim band wraps the upper rim",
            band[1][2] > 0.325 and band[1][0] > 0.162 and band[1][1] > 0.20,
            details=f"band_max={band[1]}",
        )
    logo = ctx.part_element_world_aabb(body, elem="brand_logo")
    if logo is not None:
        ctx.check(
            "brand logo on the upper front face",
            logo[1][0] > 0.155 and 0.18 < logo[0][2] < 0.24,
            details=f"logo={logo}",
        )

    # ---- open-pose checks: each drawer slides independently
    for i in range(NUM_BASKETS):
        drawer = drawers[i]
        slide = slides[i]
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: TRAVEL}):
            open_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="basket",
                min_overlap=0.05,
                name=f"drawer_{i} retains insertion at full travel",
            )
            basket_open = ctx.part_element_world_aabb(drawer, elem="basket")
            if basket_open is not None:
                ctx.check(
                    f"drawer_{i} open basket exposed beyond front wall",
                    basket_open[1][0] > 0.25,
                    details=f"basket_xmax_open={basket_open[1][0]:.4f}",
                )
        ctx.check(
            f"drawer_{i} slides forward by full travel",
            closed_pos is not None
            and open_pos is not None
            and abs((open_pos[0] - closed_pos[0]) - TRAVEL) < 1e-6
            and abs(open_pos[1] - closed_pos[1]) < 1e-9
            and abs(open_pos[2] - closed_pos[2]) < 1e-9,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    # ---- independence: opening drawer_0 does not move drawer_1
    d1_rest = ctx.part_world_position(drawers[1])
    with ctx.pose({slides[0]: TRAVEL}):
        d1_when_d0_open = ctx.part_world_position(drawers[1])
    ctx.check(
        "drawer_1 stays closed when drawer_0 opens",
        d1_rest is not None
        and d1_when_d0_open is not None
        and abs(d1_when_d0_open[0] - d1_rest[0]) < 1e-6,
        details=f"d1_rest={d1_rest}, d1_when_d0_open={d1_when_d0_open}",
    )

    return ctx.report()


object_model = build_object_model()
