from __future__ import annotations

"""Built-in single electric wall oven – French double-door variant.

Articraft brief:
- Object: built-in single electric wall oven, fascia 0.60 m wide x 0.60 m tall,
  body 0.55 m deep overall (hollow box recessed behind the front fascia).
  French double-door variant replaces the parent drop-down door with two
  side-hinged leaves that meet at the centre.
- Root/support: oven_body (hollow shell + front fascia + control strip),
  grounded at z=0; +Y into the cabinetry, -Y user-facing front.
- Parts: oven_body (root), door_leaf_0 (left, hinged on the left vertical
  edge), door_leaf_1 (right, hinged on the right vertical edge), shelf_rack
  (prismatic slide, unchanged from parent).
- Articulations:
  * body_to_door_leaf_0: REVOLUTE, hinge at left vertical edge, axis -Z so
    positive q swings the free edge outward (-Y), limits 0..pi/2.
  * body_to_door_leaf_1: REVOLUTE, hinge at right vertical edge, axis +Z so
    positive q swings the free edge outward (-Y), limits 0..pi/2.
  * body_to_shelf_rack: PRISMATIC, axis (0,-1,0), travel 0.35 m (unchanged).
- Visible geometry: matte light-gray steel fascia and shell, dark glass touch
  strip with centred clock display and flanking touch icons, two grey door
  frames each with a frosted-white glass window and a vertical brushed-
  aluminium handle bar near the centre meeting edge, dark hollow cavity with
  side rails.
- Intentional overlaps: none expected (door leaves sit in front of the fascia
  plane; the fascia plate has a cut-out opening that the leaves fill).
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

# ── overall oven dimensions (unchanged from parent) ─────────────────────
FASCIA_W = 0.60
FASCIA_H = 0.60
FASCIA_T = 0.022  # y: -0.020 .. +0.002 (embeds 2 mm into the shell front)
BODY_W = 0.56
BODY_H = 0.56  # z: 0.02 .. 0.58
BODY_D = 0.53  # y: 0.00 .. 0.53
TOTAL_D = 0.55  # fascia front (-0.02) to shell back (0.53)

OPEN_W = 0.565  # fascia door opening width
OPEN_Z0 = 0.058
OPEN_Z1 = 0.490

RACK_TRAVEL = 0.35

# ── French door leaf dimensions ──────────────────────────────────────────
LEAF_GAP = 0.003  # centre meeting gap between the two leaves
LEAF_W = (OPEN_W - LEAF_GAP) / 2.0  # ≈ 0.281
LEAF_T = 0.038
LEAF_H = OPEN_Z1 - OPEN_Z0 - 0.006  # ≈ 0.426
Z_MID = (OPEN_Z0 + OPEN_Z1) / 2.0  # ≈ 0.274
HINGE_Y = -0.020  # fascia front face plane

# window cutout inside each leaf frame
WIN_MARGIN_X = 0.030
WIN_MARGIN_Z = 0.040
WIN_W = LEAF_W - 2.0 * WIN_MARGIN_X  # ≈ 0.221
WIN_H = LEAF_H - 2.0 * WIN_MARGIN_Z  # ≈ 0.346

# vertical handle bar near the free (centre) edge
HANDLE_INSET = 0.035  # distance from free edge to bar centre
HANDLE_R = 0.010
HANDLE_LEN = 0.22
POST_R = 0.007
POST_STANDOFF = 0.025


# ── CadQuery helpers ─────────────────────────────────────────────────────
def _cyl(radius: float, length: float, axis: str, centre: tuple[float, float, float]):
    wp = cq.Workplane("XY").cylinder(length, radius)  # long axis Z
    if axis == "x":
        wp = wp.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        wp = wp.rotate((0, 0, 0), (1, 0, 0), 90)
    return wp.translate(centre)


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


def _wire_rack() -> cq.Workplane:
    rack = _cyl(0.0045, 0.40, "y", (0.230, 0.20, 0.0))
    rack = rack.union(_cyl(0.0045, 0.40, "y", (-0.230, 0.20, 0.0)))
    for y in (0.006, 0.20, 0.394):
        rack = rack.union(_cyl(0.004, 0.464, "x", (0.0, y, 0.0)))
    for i in range(9):
        x = -0.184 + 0.046 * i
        rack = rack.union(_cyl(0.003, 0.388, "y", (x, 0.20, 0.0)))
    return rack


def _door_leaf_frame(side: int) -> cq.Workplane:
    """Door-leaf frame panel with a centred window cut-out.

    *side*: +1 for the left leaf (panel extends in local +X from the hinge),
    −1 for the right leaf (extends in local −X).
    """
    panel = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H)
        .translate((side * LEAF_W / 2.0, -LEAF_T / 2.0, 0.0))
    )
    window = (
        cq.Workplane("XY")
        .box(WIN_W, LEAF_T + 0.01, WIN_H)
        .translate((side * LEAF_W / 2.0, -LEAF_T / 2.0, 0.0))
    )
    return panel.cut(window)


# ── build ────────────────────────────────────────────────────────────────
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_wall_oven_french_door")

    # materials (same palette as parent)
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

    # ── oven body (root, unchanged) ──────────────────────────────────
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
    body.visual(
        mesh_from_cadquery(_cavity_liner(), "cavity_liner"),
        material="cavity_dark",
        name="cavity_liner",
    )
    # rack support rails on the cavity side walls
    for i, sx in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.014, 0.40, 0.012)),
            origin=Origin(xyz=(sx * 0.233, 0.23, 0.199)),
            material="dark_steel",
            name=f"shelf_rail_{i}",
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

    # ── French double doors (mirrored loop) ──────────────────────────
    for i in range(2):
        side = 1 if i == 0 else -1  # +1 left, −1 right
        hinge_x = -side * (OPEN_W / 2.0)  # left: −0.2825, right: +0.2825
        axis_z = -1.0 if i == 0 else 1.0  # left: −Z, right: +Z

        leaf = model.part(f"door_leaf_{i}")

        # frame with window cut-out
        leaf.visual(
            mesh_from_cadquery(_door_leaf_frame(side), f"leaf_frame_{i}"),
            material="door_gray",
            name="leaf_frame",
        )

        # frosted glass pane (overlaps the frame window rim for seated contact)
        leaf.visual(
            Box((WIN_W + 0.004, 0.006, WIN_H + 0.004)),
            origin=Origin(xyz=(side * LEAF_W / 2.0, -LEAF_T / 2.0, 0.0)),
            material="frosted_glass",
            name="window_glass",
        )

        # vertical handle bar near the free (centre) edge
        hx = side * (LEAF_W - HANDLE_INSET)
        hy = -LEAF_T - POST_STANDOFF
        leaf.visual(
            Cylinder(radius=HANDLE_R, length=HANDLE_LEN),
            origin=Origin(xyz=(hx, hy, 0.0)),
            material="aluminum",
            name="handle_bar",
        )

        # two mounting posts (horizontal, along Y) connecting bar to face
        for j in range(2):
            pz = (HANDLE_LEN / 2.0 - 0.020) * (1 if j == 0 else -1)
            leaf.visual(
                Cylinder(radius=POST_R, length=POST_STANDOFF),
                origin=Origin(
                    xyz=(hx, -LEAF_T - POST_STANDOFF / 2.0, pz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="aluminum",
                name=f"handle_post_{j}",
            )

        # articulation: revolute about Z at the vertical hinge edge
        model.articulation(
            f"body_to_door_leaf_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leaf,
            origin=Origin(xyz=(hinge_x, HINGE_Y, Z_MID)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.5,
                lower=0.0,
                upper=math.pi / 2.0,
            ),
        )

    # ── shelf rack (unchanged from parent) ───────────────────────────
    rack = model.part("shelf_rack")
    rack.visual(
        mesh_from_cadquery(_wire_rack(), "rack_grid"),
        material="chrome_wire",
        name="rack_grid",
    )
    model.articulation(
        "body_to_shelf_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(0.0, 0.03, 0.2093)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.3, lower=0.0, upper=RACK_TRAVEL
        ),
    )

    return model


# ── tests ────────────────────────────────────────────────────────────────
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    leaf0 = object_model.get_part("door_leaf_0")
    leaf1 = object_model.get_part("door_leaf_1")
    rack = object_model.get_part("shelf_rack")
    hinge0 = object_model.get_articulation("body_to_door_leaf_0")
    hinge1 = object_model.get_articulation("body_to_door_leaf_1")
    slide = object_model.get_articulation("body_to_shelf_rack")

    # ── French-door joint contract ───────────────────────────────────
    ctx.check(
        "left leaf hinge is revolute about −Z at the left vertical edge",
        hinge0.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge0.axis) == (0.0, 0.0, -1.0)
        and abs(hinge0.origin.xyz[0] - (-OPEN_W / 2.0)) < 1e-6
        and abs(hinge0.origin.xyz[2] - Z_MID) < 1e-6
        and abs(hinge0.motion_limits.lower) < 1e-9
        and abs(hinge0.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=(
            f"axis={hinge0.axis}, origin={hinge0.origin.xyz}, "
            f"limits=({hinge0.motion_limits.lower}, {hinge0.motion_limits.upper})"
        ),
    )
    ctx.check(
        "right leaf hinge is revolute about +Z at the right vertical edge",
        hinge1.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge1.axis) == (0.0, 0.0, 1.0)
        and abs(hinge1.origin.xyz[0] - (OPEN_W / 2.0)) < 1e-6
        and abs(hinge1.origin.xyz[2] - Z_MID) < 1e-6
        and abs(hinge1.motion_limits.lower) < 1e-9
        and abs(hinge1.motion_limits.upper - math.pi / 2.0) < 1e-6,
        details=(
            f"axis={hinge1.axis}, origin={hinge1.origin.xyz}, "
            f"limits=({hinge1.motion_limits.lower}, {hinge1.motion_limits.upper})"
        ),
    )
    ctx.check(
        "rack slide is prismatic along −Y with 0.35 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, -1.0, 0.0)
        and abs(slide.motion_limits.upper - RACK_TRAVEL) < 1e-9,
        details=(
            f"axis={slide.axis}, "
            f"limits=({slide.motion_limits.lower}, {slide.motion_limits.upper})"
        ),
    )

    # ── overall envelope ─────────────────────────────────────────────
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

    # ── closed pose: both leaves inside the fascia opening ───────────
    for leaf, label in ((leaf0, "left"), (leaf1, "right")):
        ctx.expect_within(
            leaf,
            body,
            axes="xz",
            inner_elem="leaf_frame",
            outer_elem="front_fascia",
            margin=0.002,
            name=f"closed {label} leaf stays within the fascia outline",
        )
        ctx.expect_gap(
            body,
            leaf,
            axis="y",
            positive_elem="front_fascia",
            negative_elem="leaf_frame",
            max_penetration=0.002,
            max_gap=0.015,
            name=f"closed {label} leaf sits just in front of the fascia",
        )
        ctx.expect_within(
            leaf,
            leaf,
            axes="xz",
            inner_elem="window_glass",
            outer_elem="leaf_frame",
            margin=0.0,
            name=f"frosted glass captured inside {label} leaf frame",
        )

    # leaves meet at the centre when closed
    frame0_aabb = ctx.part_element_world_aabb(leaf0, elem="leaf_frame")
    frame1_aabb = ctx.part_element_world_aabb(leaf1, elem="leaf_frame")
    ctx.check(
        "leaf free edges meet near centre when closed (French-door meeting line)",
        frame0_aabb is not None
        and frame1_aabb is not None
        and frame0_aabb[1][0] > -0.01
        and frame1_aabb[0][0] < 0.01
        and frame0_aabb[1][0] < frame1_aabb[0][0] + 0.008,
        details=f"frame0={frame0_aabb}, frame1={frame1_aabb}",
    )

    # handle bars near the centre meeting line, standing proud of each leaf
    for leaf, label in ((leaf0, "left"), (leaf1, "right")):
        bar = ctx.part_element_world_aabb(leaf, elem="handle_bar")
        frame = ctx.part_element_world_aabb(leaf, elem="leaf_frame")
        ctx.check(
            f"{label} handle bar near centre and stands off in front of leaf face",
            bar is not None
            and frame is not None
            and bar[1][1] < frame[0][1] - 0.01
            and abs((bar[0][0] + bar[1][0]) / 2.0) < 0.08,
            details=f"bar={bar}, frame={frame}",
        )

    # control strip occupies the top fifth of the fascia
    glass = ctx.part_element_world_aabb(body, elem="control_glass")
    disp = ctx.part_element_world_aabb(body, elem="clock_display")
    ctx.check(
        "dark touch glass and centred display sit in the top fifth of the fascia",
        glass is not None
        and disp is not None
        and glass[0][2] > 0.48
        and disp[0][2] > 0.48
        and abs((disp[0][0] + disp[1][0]) / 2.0) < 0.005,
        details=f"glass={glass}, display={disp}",
    )

    # ── open pose: each leaf swings outward independently ────────────
    with ctx.pose({hinge0: math.pi / 2.0}):
        leaf0_open = ctx.part_world_aabb(leaf0)
        ctx.check(
            "left leaf at 90° swings outward (−Y), stays above floor",
            leaf0_open is not None
            and leaf0_open[0][1] < -0.20
            and leaf0_open[0][2] > 0.0,
            details=f"open leaf0 aabb={leaf0_open}",
        )

    with ctx.pose({hinge1: math.pi / 2.0}):
        leaf1_open = ctx.part_world_aabb(leaf1)
        ctx.check(
            "right leaf at 90° swings outward (−Y), stays above floor",
            leaf1_open is not None
            and leaf1_open[0][1] < -0.20
            and leaf1_open[0][2] > 0.0,
            details=f"open leaf1 aabb={leaf1_open}",
        )

    # both leaves can open at the same time (independent hinges)
    with ctx.pose({hinge0: math.pi / 2.0, hinge1: math.pi / 2.0}):
        both0 = ctx.part_world_aabb(leaf0)
        both1 = ctx.part_world_aabb(leaf1)
        ctx.check(
            "both leaves open simultaneously (independent hinges)",
            both0 is not None
            and both1 is not None
            and both0[0][1] < -0.20
            and both1[0][1] < -0.20,
            details=f"leaf0={both0}, leaf1={both1}",
        )

    # ── rack tests (unchanged) ───────────────────────────────────────
    ctx.expect_within(
        rack,
        body,
        axes="xy",
        margin=0.0,
        name="closed rack is fully housed inside the oven body",
    )
    ctx.expect_gap(
        rack,
        body,
        axis="z",
        negative_elem="shelf_rail_0",
        max_penetration=0.001,
        max_gap=0.001,
        name="rack wires rest on the cavity side rails",
    )
    rack_rest = ctx.part_world_position(rack)
    with ctx.pose({slide: RACK_TRAVEL}):
        rack_out = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            min_overlap=0.04,
            name="extended rack keeps retained insertion in the cavity",
        )
    ctx.check(
        "rack pulls out 0.35 m toward the user (−Y)",
        rack_rest is not None
        and rack_out is not None
        and abs((rack_rest[1] - rack_out[1]) - RACK_TRAVEL) < 1e-6,
        details=f"rest={rack_rest}, out={rack_out}",
    )

    return ctx.report()


object_model = build_object_model()
