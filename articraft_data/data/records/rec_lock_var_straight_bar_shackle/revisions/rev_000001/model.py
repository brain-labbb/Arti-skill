from __future__ import annotations

# Variant: combination padlock with straight sliding hasp bar (replaces U-shackle).
#
# Object identity: same armored body, front dial stack, and faceplate as the
# parent combination padlock, but the U-shackle is replaced by a straight
# hardened-steel hasp bar that slides across the body top on a prismatic joint.
# Two steel guide brackets retain the bar on the body.
#
# Articulation:
#   * The hasp bar slides along +X across the body top to release: PRISMATIC.
#   * Each of the four combination dials spins about its own horizontal (X)
#     axle: CONTINUOUS about X, one joint per dial.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters) — body/dials IDENTICAL to parent
# ----------------------------------------------------------------------------
BODY_W = 0.052          # body width  (X, left-right)
BODY_T = 0.020          # body thickness (Y, front-back)
BODY_H = 0.074          # body height (Z, bottom to top of housing)

FACE_RECESS = 0.0015    # depth of the black faceplate inset
ORANGE_LIP = 0.0030     # how far the orange bumper proudly wraps each edge

# Hasp bar dimensions (replaces shackle)
HASP_BAR_L = 0.066          # bar length (along X, extends past body on both sides)
HASP_BAR_W = 0.012          # bar width (along Y)
HASP_BAR_H = 0.005          # bar thickness (along Z)
HASP_TRAVEL = 0.022         # prismatic slide travel
HASP_GUIDE_SPAN = 0.030     # distance between guide brackets (matches parent shackle span)

DIAL_R = 0.0055         # combination dial radius (in the YZ plane)
DIAL_W = 0.016          # axial width of one dial (along X, its spin axle)
DIAL_GAP = 0.0008       # vertical gap between adjacent dial rims
N_DIALS = 4

TOP_Z = BODY_H          # body top plane

DIAL_PITCH = 2.0 * DIAL_R + DIAL_GAP
DIAL_STACK_CZ = 0.032                       # Z center of the 4-dial stack
DIAL_ZS = [
    DIAL_STACK_CZ + (i - (N_DIALS - 1) / 2.0) * DIAL_PITCH for i in range(N_DIALS)
]
DIAL_AXLE_Y = 0.0060                        # axle depth: rim front > face
AXLE_R = 0.0034                             # steel axle radius

POCKET_CLEAR = 0.0008
POCKET_HX = DIAL_W / 2.0 + POCKET_CLEAR
POCKET_Z_LO = DIAL_ZS[0] - DIAL_R - POCKET_CLEAR
POCKET_Z_HI = DIAL_ZS[-1] + DIAL_R + POCKET_CLEAR
POCKET_Y_BACK = DIAL_AXLE_Y - DIAL_R - POCKET_CLEAR
POCKET_Y_FRONT = 0.014


# ----------------------------------------------------------------------------
# Materials
# ----------------------------------------------------------------------------
def _register_materials(model: ArticulatedObject) -> None:
    model.material("plate_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("body_orange", rgba=(0.86, 0.40, 0.10, 1.0))
    model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("dial_black", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("dial_band", rgba=(0.30, 0.30, 0.32, 1.0))
    model.material("rivet_steel", rgba=(0.70, 0.72, 0.75, 1.0))


# ----------------------------------------------------------------------------
# Body geometry (CadQuery) — IDENTICAL to parent except hasp channel on top
# ----------------------------------------------------------------------------
def _orange_body_shell() -> cq.Workplane:
    """Orange rubberized outer body with beveled hexagon-ish lower corners."""
    hw = BODY_W / 2.0
    chamf = 0.012
    pts = [
        (-hw, chamf),
        (-hw + chamf, 0.0),
        (hw - chamf, 0.0),
        (hw, chamf),
        (hw, BODY_H - chamf),
        (hw - chamf, BODY_H),
        (-hw + chamf, BODY_H),
        (-hw, BODY_H - chamf),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BODY_T)
        .translate((0.0, BODY_T / 2.0, 0.0))
    )
    shell = shell.edges("|Y").fillet(0.0035)
    return shell


def _dial_pocket_cutter() -> cq.Workplane:
    """Open window pocket through the body front for the vertical dial stack."""
    return (
        cq.Workplane("XY")
        .box(
            2.0 * POCKET_HX,
            POCKET_Y_FRONT - POCKET_Y_BACK,
            POCKET_Z_HI - POCKET_Z_LO,
            centered=(True, True, True),
        )
        .translate(
            (
                0.0,
                (POCKET_Y_FRONT + POCKET_Y_BACK) / 2.0,
                (POCKET_Z_HI + POCKET_Z_LO) / 2.0,
            )
        )
    )


def _black_faceplate() -> cq.Workplane:
    """Black plastic faceplate inset into the front of the body."""
    hw = BODY_W / 2.0
    inset = ORANGE_LIP + 0.0005
    plate_hw = hw - inset
    plate_top = BODY_H - inset
    plate_bot = inset
    plate_thk = 0.004
    front_y = BODY_T / 2.0
    plate = (
        cq.Workplane("XY")
        .box(2.0 * plate_hw, plate_thk, plate_top - plate_bot, centered=(True, True, True))
        .translate((0.0, front_y - plate_thk / 2.0, (plate_top + plate_bot) / 2.0))
    )
    plate = plate.edges("|Y").fillet(0.0035)
    plate = plate.cut(_dial_pocket_cutter())
    return plate


def _dial_window_frame() -> cq.Workplane:
    """Raised dark bezel around the vertical four-dial window."""
    win_w = 2.0 * POCKET_HX
    win_h = POCKET_Z_HI - POCKET_Z_LO
    cz = (POCKET_Z_LO + POCKET_Z_HI) / 2.0
    front0 = BODY_T / 2.0
    front1 = front0 + 0.0020
    outer = (
        cq.Workplane("XY")
        .box(win_w + 0.006, (front1 - front0) + 0.003, win_h + 0.006,
             centered=(True, True, True))
        .translate((0.0, (front0 - 0.003 + front1) / 2.0, cz))
    )
    inner = (
        cq.Workplane("XY")
        .box(win_w, 0.012, win_h, centered=(True, True, True))
        .translate((0.0, (front0 + front1) / 2.0, cz))
    )
    return outer.cut(inner)


def _corner_rivets() -> cq.Workplane:
    """Four steel corner rivets on the faceplate."""
    hw = BODY_W / 2.0 - ORANGE_LIP - 0.005
    z_top = BODY_H - ORANGE_LIP - 0.006
    z_bot = ORANGE_LIP + 0.006
    rivets = None
    for sx in (-1.0, 1.0):
        for sz in (z_bot, z_top):
            r = (
                cq.Workplane("XZ", origin=(sx * hw, BODY_T / 2.0, sz))
                .cylinder(0.0015, 0.0016, direct=(0, 1, 0), centered=(True, True, False))
            )
            rivets = r if rivets is None else rivets.union(r)
    return rivets


def _hasp_guide_channel() -> cq.Workplane:
    """Shallow groove across the body top for the hasp bar to ride in."""
    channel_w = HASP_BAR_W + 0.002
    channel_l = BODY_W + 0.006
    channel_d = 0.0015
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_H - channel_d))
        .box(channel_l, channel_w, channel_d + 0.001, centered=(True, True, False))
    )


def _build_body_mesh_orange() -> object:
    shell = _orange_body_shell()
    shell = shell.cut(_hasp_guide_channel())
    shell = shell.cut(_dial_pocket_cutter())
    return shell


def _build_hasp_guide_brackets() -> object:
    """Two steel guide brackets on the body top that retain the hasp bar.

    Each bracket is a top plate above the bar channel, supported by two pillars
    on either side of the bar. The pillars extend slightly into the body shell
    for mesh connectivity. The bar slides freely through the channel beneath.
    """
    plate_w = 0.012       # along X
    plate_l = HASP_BAR_W + 0.008  # along Y, wider than the bar
    plate_h = 0.002       # thickness
    plate_z = BODY_H + HASP_BAR_H  # bottom of bracket plate = bar top
    pillar_h = HASP_BAR_H + 0.001  # extends 1mm into body for connectivity
    pillar_size = 0.003

    brackets = None
    for sx in (-1.0, 1.0):
        bx = sx * (HASP_GUIDE_SPAN / 2.0)
        # Top plate
        b = (
            cq.Workplane("XY", origin=(bx, 0.0, plate_z))
            .box(plate_w, plate_l, plate_h, centered=(True, True, False))
        )
        # Two pillars on either side of the bar channel
        for sy in (-1.0, 1.0):
            pillar_y = sy * (HASP_BAR_W / 2.0 + pillar_size / 2.0 + 0.0005)
            pillar = (
                cq.Workplane("XY", origin=(bx, pillar_y, BODY_H - 0.001))
                .box(pillar_size, pillar_size, pillar_h, centered=(True, True, False))
            )
            b = b.union(pillar)
        brackets = b if brackets is None else brackets.union(b)
    return brackets


def _build_dial_axles() -> object:
    """Internal axle pins the four dials spin on."""
    axle_half = (DIAL_W + 0.012) / 2.0
    axles = None
    for cz in DIAL_ZS:
        a = (
            cq.Workplane("YZ", origin=(0.0, DIAL_AXLE_Y, cz))
            .circle(AXLE_R)
            .extrude(axle_half, both=True)
        )
        axles = a if axles is None else axles.union(a)
    return axles


# ----------------------------------------------------------------------------
# Hasp bar geometry (CadQuery) — straight sliding bar
# ----------------------------------------------------------------------------
def _build_hasp_bar_mesh() -> object:
    """Straight hardened-steel hasp bar with rounded ends and a latch notch.

    Authored with bottom surface at z=0 so the joint origin at the body top
    places the bar directly on the body surface.
    """
    # Main bar body
    bar = (
        cq.Workplane("XY")
        .box(HASP_BAR_L, HASP_BAR_W, HASP_BAR_H, centered=(True, True, False))
    )
    # Round the vertical edges for finished bar ends (stadium profile)
    bar = bar.edges("|Z").fillet(0.004)
    # Latch notch at the +X end — a rectangular cutout for the keeper
    notch_w = 0.007
    notch_h = HASP_BAR_W * 0.45
    notch = (
        cq.Workplane("XY", origin=(HASP_BAR_L / 2.0 - 0.004, 0.0, HASP_BAR_H / 2.0))
        .box(notch_w, notch_h, HASP_BAR_H + 0.002, centered=(True, True, True))
    )
    bar = bar.cut(notch)
    return bar


# ----------------------------------------------------------------------------
# Dial geometry (CadQuery) — IDENTICAL to parent
# ----------------------------------------------------------------------------
def _build_dial_mesh() -> object:
    """A single combination dial: a knurled wheel with a raised center band."""
    wheel = (
        cq.Workplane("YZ")
        .circle(DIAL_R)
        .extrude(DIAL_W / 2.0, both=True)
    )
    n_teeth = 28
    cutter = None
    groove_depth = 0.0006
    for i in range(n_teeth):
        a = 2.0 * math.pi * i / n_teeth
        gy = (DIAL_R - groove_depth / 2.0) * math.cos(a)
        gz = (DIAL_R - groove_depth / 2.0) * math.sin(a)
        g = (
            cq.Workplane("YZ", origin=(0.0, gy, gz))
            .rect(0.0006, 0.0012)
            .extrude(DIAL_W / 2.0 + 0.0002, both=True)
            .rotate((0, 0, 0), (1, 0, 0), math.degrees(a))
        )
        cutter = g if cutter is None else cutter.union(g)
    wheel = wheel.cut(cutter)
    flat = (
        cq.Workplane("YZ", origin=(0.0, 0.0, DIAL_R + 0.0006))
        .rect(2.0 * DIAL_R, 0.0024)
        .extrude(DIAL_W / 2.0 + 0.0005, both=True)
    )
    wheel = wheel.cut(flat)
    bore = (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))
        .circle(0.0040)
        .extrude(DIAL_W, both=True)
    )
    wheel = wheel.cut(bore)
    return wheel


# ----------------------------------------------------------------------------
# Model assembly
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_padlock_hasp")
    _register_materials(model)

    # ---- Body (root) — IDENTICAL except hasp guide channel + brackets ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_mesh_orange(), "body_shell"),
        material="body_orange",
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_black_faceplate(), "faceplate"),
        material="plate_black",
        name="faceplate",
    )
    body.visual(
        mesh_from_cadquery(_dial_window_frame(), "dial_frame"),
        material="dial_black",
        name="dial_frame",
    )
    body.visual(
        mesh_from_cadquery(_corner_rivets(), "rivets"),
        material="rivet_steel",
        name="rivets",
    )
    body.visual(
        mesh_from_cadquery(_build_dial_axles(), "dial_axles"),
        material="rivet_steel",
        name="dial_axles",
    )
    # Hasp guide brackets (new for this variant — retain the sliding bar)
    body.visual(
        mesh_from_cadquery(_build_hasp_guide_brackets(), "hasp_brackets"),
        material="rivet_steel",
        name="hasp_brackets",
    )

    # ---- Hasp bar (prismatic slide along +X across body top) ----
    hasp_bar = model.part("hasp_bar")
    hasp_bar.visual(
        mesh_from_cadquery(_build_hasp_bar_mesh(), "hasp_bar"),
        material="steel",
        name="hasp_bar",
    )
    # The hasp bar slides along +X across the body top. Positive q slides the
    # bar toward +X to release the lock. The bar sits on the body top plane
    # and is retained by the guide brackets above.
    model.articulation(
        "body_to_hasp",
        ArticulationType.PRISMATIC,
        parent=body,
        child=hasp_bar,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0,
                                   upper=HASP_TRAVEL),
    )

    # ---- Four combination dials (IDENTICAL to parent) ----
    for idx, cz in enumerate(DIAL_ZS):
        dial = model.part(f"dial_{idx + 1}")
        dial.visual(
            mesh_from_cadquery(_build_dial_mesh(), f"dial_{idx + 1}_wheel"),
            material="dial_black",
            name=f"dial_{idx + 1}_wheel",
        )
        dial.visual(
            Cylinder(radius=DIAL_R * 0.985, length=DIAL_W * 0.42),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dial_band",
            name=f"dial_{idx + 1}_band",
        )
        model.articulation(
            f"dial_{idx + 1}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(0.0, DIAL_AXLE_Y, cz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=4.0),
        )

    return model


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hasp_bar = object_model.get_part("hasp_bar")
    hasp_joint = object_model.get_articulation("body_to_hasp")

    # --- Joint type / axis contract for the hasp bar ---
    ctx.check(
        "hasp bar joint is a prismatic slide",
        hasp_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={hasp_joint.articulation_type}",
    )
    ctx.check(
        "hasp bar slides along horizontal X axis",
        abs(hasp_joint.axis[0]) > 0.99
        and abs(hasp_joint.axis[1]) < 0.01
        and abs(hasp_joint.axis[2]) < 0.01,
        details=f"axis={hasp_joint.axis}",
    )

    # --- Hasp bar sits on top of the body shell ---
    body_aabb = ctx.part_world_aabb(body)
    hasp_aabb = ctx.part_world_aabb(hasp_bar)
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "hasp bar sits on the body shell top",
        hasp_aabb is not None and shell_aabb is not None
        and hasp_aabb[0][2] >= shell_aabb[1][2] - 0.002,
        details=f"hasp_bottom={hasp_aabb[0][2] if hasp_aabb else None}, "
                f"shell_top={shell_aabb[1][2] if shell_aabb else None}",
    )
    # Hasp bar overlaps the body footprint in XY (it sits on top)
    ctx.expect_overlap(hasp_bar, body, axes="xy", min_overlap=0.004,
                       name="hasp bar overlaps body footprint in XY")

    # The hasp bar rides in the body guide channel; the seated sliding fit is
    # intentional. Allow overlap scoped to the bar/channel interface.
    ctx.allow_overlap(
        body, hasp_bar,
        reason="Hasp bar rides in the body top guide channel beneath the "
               "retaining brackets; seated sliding fit is intentional.",
    )

    # --- Slide moves the bar sideways; travel is correct ---
    rest_pos = ctx.part_world_position(hasp_bar)
    with ctx.pose({hasp_joint: HASP_TRAVEL}):
        open_pos = ctx.part_world_position(hasp_bar)
    ctx.check(
        "prismatic slide moves the hasp bar by the joint travel",
        rest_pos is not None and open_pos is not None
        and abs((open_pos[0] - rest_pos[0]) - HASP_TRAVEL) < 1e-4,
        details=f"rest_x={rest_pos[0] if rest_pos else None}, "
                f"open_x={open_pos[0] if open_pos else None}",
    )
    ctx.check(
        "slide is purely horizontal (no vertical or lateral drift)",
        rest_pos is not None and open_pos is not None
        and abs(open_pos[2] - rest_pos[2]) < 1e-5
        and abs(open_pos[1] - rest_pos[1]) < 1e-5,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # --- Hasp bar extends past the body on at least one side at rest ---
    ctx.check(
        "hasp bar extends past the body on at least one side at rest",
        hasp_aabb is not None and body_aabb is not None
        and (hasp_aabb[1][0] > body_aabb[1][0] + 0.002
             or hasp_aabb[0][0] < body_aabb[0][0] - 0.002),
        details=f"hasp_x=[{hasp_aabb[0][0] if hasp_aabb else None}, "
                f"{hasp_aabb[1][0] if hasp_aabb else None}], "
                f"body_x=[{body_aabb[0][0] if body_aabb else None}, "
                f"{body_aabb[1][0] if body_aabb else None}]",
    )

    # --- At full slide, the latch end extends further past the body ---
    with ctx.pose({hasp_joint: HASP_TRAVEL}):
        slide_aabb = ctx.part_world_aabb(hasp_bar)
    ctx.check(
        "hasp bar latch end extends further past the body at full slide",
        slide_aabb is not None and body_aabb is not None
        and slide_aabb[1][0] > body_aabb[1][0] + HASP_TRAVEL * 0.8,
        details=f"slide_max_x={slide_aabb[1][0] if slide_aabb else None}, "
                f"body_max_x={body_aabb[1][0] if body_aabb else None}",
    )

    # --- Four dials present, stacked vertically along Z, and rotate ---
    dials = [object_model.get_part(f"dial_{i + 1}") for i in range(N_DIALS)]
    dial_joints = [object_model.get_articulation(f"dial_{i + 1}") for i in range(N_DIALS)]
    for i, dj in enumerate(dial_joints):
        ctx.check(
            f"dial_{i + 1} joint is continuous about horizontal X",
            dj.articulation_type == ArticulationType.CONTINUOUS
            and abs(dj.axis[0]) > 0.99,
            details=f"type={dj.articulation_type}, axis={dj.axis}",
        )

    zs = []
    for d in dials:
        p = ctx.part_world_position(d)
        zs.append(p[2] if p else None)
    ctx.check(
        "four dials stacked vertically in increasing Z order",
        all(z is not None for z in zs)
        and all(zs[i] < zs[i + 1] for i in range(N_DIALS - 1)),
        details=f"dial_z={zs}",
    )

    front_face_y = BODY_T / 2.0
    for i, d in enumerate(dials):
        d_aabb = ctx.part_world_aabb(d)
        ctx.check(
            f"dial_{i + 1} rim protrudes past the front face window",
            d_aabb is not None and d_aabb[1][1] >= front_face_y - 1e-5,
            details=f"dial_front_y={d_aabb[1][1] if d_aabb else None}, face_y={front_face_y}",
        )

    for i, d in enumerate(dials):
        ctx.allow_overlap(
            body, d,
            reason="Dial wheel is captured on its body axle inside the window "
                   "pocket; the rear of the wheel sits behind the front face.",
        )
        ctx.expect_overlap(d, body, axes="z", min_overlap=0.004,
                           name=f"dial_{i + 1} seats inside body window in Z")
        ctx.expect_overlap(d, body, axes="y", min_overlap=0.004,
                           name=f"dial_{i + 1} captured in body depth")

    d1 = dials[0]
    rim_rest = ctx.part_element_world_aabb(d1, elem="dial_1_wheel")
    with ctx.pose({dial_joints[0]: math.pi / 2.0}):
        rim_turn = ctx.part_element_world_aabb(d1, elem="dial_1_wheel")
    ctx.check(
        "dial_1 rotates about its axle",
        rim_rest is not None and rim_turn is not None
        and (abs(rim_turn[1][1] - rim_rest[1][1]) > 1e-4
             or abs(rim_turn[1][2] - rim_rest[1][2]) > 1e-4),
        details=f"rest_max={rim_rest[1] if rim_rest else None}, "
                f"turn_max={rim_turn[1] if rim_turn else None}",
    )

    return ctx.report()


object_model = build_object_model()
