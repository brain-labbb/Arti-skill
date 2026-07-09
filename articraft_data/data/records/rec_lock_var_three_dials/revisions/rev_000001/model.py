from __future__ import annotations

# Realistic articulated 3-dial combination padlock.
#
# Object identity (from reference image): a resettable combination padlock with
# a black plastic faceplate, orange rubberized edge bumpers wrapping the body,
# a polished steel "HARDENED" U-shackle on top, "LOCK" branding, and three
# combination number dials stacked VERTICALLY (one above another) in a recessed
# window on the front, their knurled rims protruding through the window.
#
# Articulation (reasoned from physics):
#   * The shackle releases by lifting straight up out of its body bores once
#     the correct code is dialed; the long leg stays captive in its deep bore.
#     -> PRISMATIC along +Z (18 mm travel).
#   * Each of the three combination dials spins about its own horizontal (X)
#     axle; the three axles are stacked one above another along Z behind the
#     front window. -> CONTINUOUS about X, one joint per dial.

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
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
BODY_W = 0.052          # body width  (X, left-right)
BODY_T = 0.020          # body thickness (Y, front-back)
BODY_H = 0.074          # body height (Z, bottom to top of housing)

FACE_RECESS = 0.0015    # depth of the black faceplate inset
ORANGE_LIP = 0.0030     # how far the orange bumper proudly wraps each edge

SHACKLE_WIRE_R = 0.0042            # shackle bar radius
SHACKLE_SPAN = 0.030               # center-to-center distance between the two legs
SHACKLE_CLEAR = 0.022              # clear height of the U opening above the body
LONG_LEG_BURY = 0.026              # how deep the long (captive) leg sits in the body
SHORT_LEG_BURY = 0.013             # how deep the short (latch) leg sits in the body
SHACKLE_TRAVEL = 0.018             # prismatic release lift (short leg fully clears)

DIAL_R = 0.0055         # combination dial radius (in the YZ plane)
DIAL_W = 0.016          # axial width of one dial (along X, its spin axle)
DIAL_GAP = 0.0008       # vertical gap between adjacent dial rims
N_DIALS = 3

# Pivot/captive leg sits toward -X, latch leg toward +X.
LEG_X = SHACKLE_SPAN / 2.0
TOP_Z = BODY_H          # body top plane

# Dials are stacked vertically along Z on the body front. Every dial spins on
# its own horizontal X axle at the same Y depth; the axle Y is chosen so the
# dial rim protrudes past the faceplate front (visible/thumbable from outside).
DIAL_PITCH = 2.0 * DIAL_R + DIAL_GAP
DIAL_STACK_CZ = 0.032                       # Z center of the 3-dial stack
DIAL_ZS = [
    DIAL_STACK_CZ + (i - (N_DIALS - 1) / 2.0) * DIAL_PITCH for i in range(N_DIALS)
]
DIAL_AXLE_Y = 0.0060                        # axle depth: rim front = 0.0115 > face 0.010
AXLE_R = 0.0034                             # steel axle radius (dial bore is 0.0040)

# Window pocket cut through the front of the body shell + faceplate so the
# dials are visible and spin freely (>= POCKET_CLEAR to every static wall).
POCKET_CLEAR = 0.0008
POCKET_HX = DIAL_W / 2.0 + POCKET_CLEAR                     # half width in X
POCKET_Z_LO = DIAL_ZS[0] - DIAL_R - POCKET_CLEAR            # bottom wall
POCKET_Z_HI = DIAL_ZS[-1] + DIAL_R + POCKET_CLEAR           # top wall
POCKET_Y_BACK = DIAL_AXLE_Y - DIAL_R - POCKET_CLEAR         # rear wall (inside body)
POCKET_Y_FRONT = 0.014                                      # past every front surface


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
# Body geometry (CadQuery)
# ----------------------------------------------------------------------------
def _orange_body_shell() -> cq.Workplane:
    """Orange rubberized outer body with beveled hexagon-ish lower corners.

    The reference shows a softly chamfered rectangular body whose corners are
    cut at 45 degrees, giving a stubby hexagonal silhouette, all wrapped in an
    orange bumper.
    """
    hw = BODY_W / 2.0
    chamf = 0.012  # corner cut size
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
        .extrude(BODY_T)  # extrudes along -Y; recenter below
        .translate((0.0, BODY_T / 2.0, 0.0))
    )
    # Round the front/back edges for a soft molded look.
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
    """Black plastic faceplate inset into the front of the body.

    Spans y in [BODY_T/2 - 0.004, BODY_T/2] so it reads as a slightly proud
    black panel sitting flush on the orange front, wrapped by the orange lip.
    The dial window is cut clean through it.
    """
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
    """Raised dark bezel around the vertical three-dial window (the dial cage).

    A 3 mm-wide rectangular ring standing proud of the faceplate; its inner
    opening matches the pocket footprint so it never clips the spinning dials.
    """
    win_w = 2.0 * POCKET_HX
    win_h = POCKET_Z_HI - POCKET_Z_LO
    cz = (POCKET_Z_LO + POCKET_Z_HI) / 2.0
    front0 = BODY_T / 2.0          # faceplate front plane
    front1 = front0 + 0.0020       # bezel front plane
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
    """Four steel corner rivets on the faceplate, as seen in the image."""
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


def _shackle_bores() -> cq.Workplane:
    """Two vertical bores in the top of the body for the shackle legs."""
    bore_r = SHACKLE_WIRE_R + 0.0006
    bores = None
    for sx in (-1.0, 1.0):
        b = (
            cq.Workplane("XY", origin=(sx * LEG_X, 0.0, BODY_H - 0.0))
            .cylinder(0.030, bore_r, centered=(True, True, False))
            .translate((0.0, 0.0, -0.030 + 0.0))
        )
        bores = b if bores is None else bores.union(b)
    return bores


def _shackle_boss_rings() -> cq.Workplane:
    """Small raised boss collars around each shackle bore on the body top.

    These are realistic close-fit collars that guide the shackle legs and
    provide visible contact between the body top surface and the shackle.
    The inner bore matches the shackle leg radius exactly (zero clearance
    contact fit), while the outer ring stands slightly proud of the body top.
    """
    collar_r_inner = SHACKLE_WIRE_R + 1e-7     # near-zero clearance for contact
    collar_r_outer = SHACKLE_WIRE_R + 0.002    # 2 mm wall thickness
    collar_h = 0.001                           # 1 mm proud of body top
    rings = None
    for sx in (-1.0, 1.0):
        outer = (
            cq.Workplane("XY", origin=(sx * LEG_X, 0.0, BODY_H))
            .circle(collar_r_outer)
            .extrude(collar_h)
        )
        inner_cut = (
            cq.Workplane("XY", origin=(sx * LEG_X, 0.0, BODY_H - 0.001))
            .circle(collar_r_inner)
            .extrude(collar_h + 0.002)
        )
        ring = outer.cut(inner_cut)
        rings = ring if rings is None else rings.union(ring)
    return rings


def _build_body_mesh_orange() -> object:
    shell = _orange_body_shell()
    # Cut the shackle bores into the top and the dial window into the front.
    shell = shell.cut(_shackle_bores())
    shell = shell.cut(_dial_pocket_cutter())
    return shell


def _build_dial_axles() -> object:
    """Internal axle pins the three dials spin on.

    One steel cylinder per dial, running along X through that dial's center.
    Each extends past the window pocket into the solid shell on both sides so
    it is solidly carried by the body; every dial nests over its own axle with
    a 0.6 mm radial clearance (captured running fit).
    """
    axle_half = (DIAL_W + 0.012) / 2.0  # reach into both side walls
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
# Shackle geometry (CadQuery) - U-shaped steel bar
# ----------------------------------------------------------------------------
def _build_shackle_mesh() -> object:
    """U-shaped shackle authored in the LONG-LEG local frame.

    The long (captive) leg axis is the local Z axis at the origin. The crown
    arches up to +Z and over to the short leg at +X (distance SHACKLE_SPAN).
    Buried lengths extend below z=0 so each leg seats inside its body bore.

    Built as exact primitives (two straight cylinders + a revolved half-torus
    crown that is tangent to both legs) so the legs are perfectly straight and
    stay inside their bores - a swept spline would bow the legs into the body.
    """
    arch_top = SHACKLE_CLEAR + SHACKLE_WIRE_R
    span = SHACKLE_SPAN
    crown_cy = arch_top - span / 2.0  # Z where the crown arc meets the legs

    long_leg = (
        cq.Workplane("XY", origin=(0.0, 0.0, -LONG_LEG_BURY))
        .circle(SHACKLE_WIRE_R)
        .extrude(LONG_LEG_BURY + crown_cy)
    )
    short_leg = (
        cq.Workplane("XY", origin=(span, 0.0, -SHORT_LEG_BURY))
        .circle(SHACKLE_WIRE_R)
        .extrude(SHORT_LEG_BURY + crown_cy)
    )
    # Half torus: revolve the long-leg cross-section 180 degrees about the
    # Y-direction axis through the crown center (span/2, *, crown_cy).
    crown = (
        cq.Workplane("XY", origin=(0.0, 0.0, crown_cy))
        .circle(SHACKLE_WIRE_R)
        .revolve(180.0, (span / 2.0, 0.0), (span / 2.0, 1.0))
    )
    return long_leg.union(crown).union(short_leg)


# ----------------------------------------------------------------------------
# Dial geometry (CadQuery) - knurled number wheel
# ----------------------------------------------------------------------------
def _build_dial_mesh() -> object:
    """A single combination dial: a knurled wheel with a raised center band.

    Authored centered on the origin with the spin axis along local X.
    """
    # Base wheel: cylinder whose axis is X.
    wheel = (
        cq.Workplane("YZ")
        .circle(DIAL_R)
        .extrude(DIAL_W / 2.0, both=True)
    )
    # Knurl: cut shallow axial grooves around the rim so it reads as grippy.
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
    # Index flat at the top (+Z): a milled flat that breaks rotational symmetry
    # so the printed number facing the window is unambiguous and the wheel reads
    # as a real number ring with a current-digit position.
    flat = (
        cq.Workplane("YZ", origin=(0.0, 0.0, DIAL_R + 0.0006))
        .rect(2.0 * DIAL_R, 0.0024)
        .extrude(DIAL_W / 2.0 + 0.0005, both=True)
    )
    wheel = wheel.cut(flat)
    # Axle bore: the dial spins on the body's internal axle pin that passes
    # through this center bore (captured running fit, 0.6 mm clearance).
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
    model = ArticulatedObject(name="combination_padlock")
    _register_materials(model)

    # ---- Body (root) ----
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
    # Internal axle pins the dials spin on (carried by both side walls).
    body.visual(
        mesh_from_cadquery(_build_dial_axles(), "dial_axles"),
        material="rivet_steel",
        name="dial_axles",
    )
    # Shackle bore boss rings on body top (close-fit collars for shackle legs).
    body.visual(
        mesh_from_cadquery(_shackle_boss_rings(), "shackle_bosses"),
        material="body_orange",
        name="shackle_bosses",
    )

    # ---- Shackle (prismatic release lift along +Z) ----
    shackle = model.part("shackle")
    shackle.visual(
        mesh_from_cadquery(_build_shackle_mesh(), "shackle_bar"),
        material="steel",
        name="shackle_bar",
    )
    # The shackle releases by lifting straight up out of its bores: positive q
    # raises the whole U by up to SHACKLE_TRAVEL. The short leg (buried 13 mm)
    # fully clears the body top at full travel while the long leg (buried
    # 26 mm in a 30 mm bore) stays captive, exactly like a real padlock.
    model.articulation(
        "body_to_shackle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-LEG_X, 0.0, TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.2, lower=0.0,
                                   upper=SHACKLE_TRAVEL),
    )

    # ---- Three combination dials (stacked vertically, continuous about X) ----
    for idx, cz in enumerate(DIAL_ZS):
        dial = model.part(f"dial_{idx + 1}")
        dial.visual(
            mesh_from_cadquery(_build_dial_mesh(), f"dial_{idx + 1}_wheel"),
            material="dial_black",
            name=f"dial_{idx + 1}_wheel",
        )
        # A thin band visual to read as the printed number ring.
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
    shackle = object_model.get_part("shackle")
    shackle_joint = object_model.get_articulation("body_to_shackle")

    # --- Joint type / axis contract for the shackle ---
    ctx.check(
        "shackle joint is a prismatic release lift",
        shackle_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={shackle_joint.articulation_type}",
    )
    ctx.check(
        "shackle lifts along vertical (Z) axis",
        abs(shackle_joint.axis[2]) > 0.99
        and abs(shackle_joint.axis[0]) < 0.01
        and abs(shackle_joint.axis[1]) < 0.01,
        details=f"axis={shackle_joint.axis}",
    )

    # --- Shackle is mounted on top of the body, not floating ---
    body_aabb = ctx.part_world_aabb(body)
    sh_aabb = ctx.part_world_aabb(shackle)
    ctx.check(
        "shackle crown rises above the body top",
        sh_aabb is not None and body_aabb is not None
        and sh_aabb[1][2] > body_aabb[1][2] + 0.010,
        details=f"shackle_top={sh_aabb[1][2] if sh_aabb else None}, body_top={body_aabb[1][2] if body_aabb else None}",
    )
    # Both legs seat inside the body footprint at rest (captured in bores).
    ctx.expect_overlap(shackle, body, axes="xy", min_overlap=0.004,
                       name="shackle legs seat into body bores")
    ctx.expect_overlap(shackle, body, axes="z", min_overlap=0.004,
                       name="shackle legs buried in body top")
    # Legs nest inside the body bores -> intentional capture overlap.
    ctx.allow_overlap(
        body, shackle,
        reason="Shackle legs ride inside the body bores; this seated/captured fit is intentional.",
    )

    # --- Full lift raises the shackle straight up; long leg stays captive ---
    rest_aabb = ctx.part_world_aabb(shackle)
    with ctx.pose({shackle_joint: SHACKLE_TRAVEL}):
        open_aabb = ctx.part_world_aabb(shackle)
    ctx.check(
        "release lift raises the shackle by the joint travel",
        rest_aabb is not None and open_aabb is not None
        and abs((open_aabb[1][2] - rest_aabb[1][2]) - SHACKLE_TRAVEL) < 1e-4
        and abs((open_aabb[0][2] - rest_aabb[0][2]) - SHACKLE_TRAVEL) < 1e-4,
        details=f"rest_top={rest_aabb[1][2] if rest_aabb else None}, open_top={open_aabb[1][2] if open_aabb else None}",
    )
    ctx.check(
        "lift is purely vertical (no sideways sweep through the body)",
        rest_aabb is not None and open_aabb is not None
        and abs(open_aabb[0][0] - rest_aabb[0][0]) < 1e-5
        and abs(open_aabb[1][0] - rest_aabb[1][0]) < 1e-5
        and abs(open_aabb[0][1] - rest_aabb[0][1]) < 1e-5
        and abs(open_aabb[1][1] - rest_aabb[1][1]) < 1e-5,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )
    # Short leg bottom (rest: TOP_Z - SHORT_LEG_BURY) clears the body top at
    # full travel; the long leg bottom remains below the body top (captive).
    short_leg_bottom_open = TOP_Z - SHORT_LEG_BURY + SHACKLE_TRAVEL
    long_leg_bottom_open = TOP_Z - LONG_LEG_BURY + SHACKLE_TRAVEL
    ctx.check(
        "short leg fully clears the body top at full lift",
        body_aabb is not None and short_leg_bottom_open > body_aabb[1][2] + 0.003,
        details=f"short_leg_bottom_open={short_leg_bottom_open:.4f}, body_top={body_aabb[1][2] if body_aabb else None}",
    )
    ctx.check(
        "long leg stays captive in its bore at full lift",
        body_aabb is not None and long_leg_bottom_open < body_aabb[1][2] - 0.004,
        details=f"long_leg_bottom_open={long_leg_bottom_open:.4f}, body_top={body_aabb[1][2] if body_aabb else None}",
    )

    # --- Three dials present, stacked vertically along Z, and rotate ---
    dials = [object_model.get_part(f"dial_{i + 1}") for i in range(N_DIALS)]
    joints = [object_model.get_articulation(f"dial_{i + 1}") for i in range(N_DIALS)]
    for i, dj in enumerate(joints):
        ctx.check(
            f"dial_{i + 1} joint is continuous about horizontal X",
            dj.articulation_type == ArticulationType.CONTINUOUS
            and abs(dj.axis[0]) > 0.99,
            details=f"type={dj.articulation_type}, axis={dj.axis}",
        )

    # Dials stacked bottom-to-top along Z in order, sharing the same X/Y.
    zs = []
    for d in dials:
        p = ctx.part_world_position(d)
        zs.append(p[2] if p else None)
    ctx.check(
        "three dials stacked vertically in increasing Z order",
        all(z is not None for z in zs)
        and all(zs[i] < zs[i + 1] for i in range(N_DIALS - 1)),
        details=f"dial_z={zs}",
    )

    # Dial rims protrude past the faceplate front so they are visible and
    # thumbable from outside (the photo's dominant feature).
    front_face_y = BODY_T / 2.0
    for i, d in enumerate(dials):
        d_aabb = ctx.part_world_aabb(d)
        ctx.check(
            f"dial_{i + 1} rim protrudes past the front face window",
            d_aabb is not None and d_aabb[1][1] >= front_face_y - 1e-5,
            details=f"dial_front_y={d_aabb[1][1] if d_aabb else None}, face_y={front_face_y}",
        )

    # Each dial is captured in the body window pocket on its own axle. This
    # seated/captured fit is intentional; allow it and prove retained insertion.
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

    # Rotating a dial actually spins it (the index flat moves off top).
    d1 = dials[0]
    rim_rest = ctx.part_element_world_aabb(d1, elem="dial_1_wheel")
    with ctx.pose({joints[0]: math.pi / 2.0}):
        rim_turn = ctx.part_element_world_aabb(d1, elem="dial_1_wheel")
    ctx.check(
        "dial_1 rotates about its axle",
        rim_rest is not None and rim_turn is not None
        and (abs(rim_turn[1][1] - rim_rest[1][1]) > 1e-4
             or abs(rim_turn[1][2] - rim_rest[1][2]) > 1e-4),
        details=f"rest_max={rim_rest[1] if rim_rest else None}, turn_max={rim_turn[1] if rim_turn else None}",
    )

    return ctx.report()


object_model = build_object_model()
