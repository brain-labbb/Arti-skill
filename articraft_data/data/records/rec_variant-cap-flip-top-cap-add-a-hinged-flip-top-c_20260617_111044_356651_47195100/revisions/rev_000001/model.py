from __future__ import annotations

"""Classic disposable flint pocket lighter (BIC style), standing upright.

Variant: flip_top_cap – adds a hinged flip-top cap over the nozzle/hood that
swings up to expose the flame (REVOLUTE about X at the rear top edge),
windproof-lighter style.

Frame conventions (object-intrinsic):
- Z is up; the lighter stands on z = 0.
- Y is the long plan axis (~0.025 m): +Y is the flame/notch end ("front"),
  -Y is the thumb-lever end ("rear").
- X is the short plan axis (~0.012 m): the spark-wheel axle runs left-right
  along X, exactly as on the real lighter (axle perpendicular to the broad
  faces, held by the two chrome cheek plates).

Parts:
- body (root): translucent royal-blue stadium-plan fuel reservoir, hollow at
  the open top (visible wall rim), grey valve deck recessed inside, chrome
  flame nozzle + flint tube rising from the deck, polished chrome hood band
  wrapping the top 0.015 m (front flame notch, side vent perforations, cut-down
  rear edge for the lever), two chrome cheek plates flanking the wheel, and a
  steel hinge pin captured through the hood for the flip cap.
- spark_wheel: knurled steel wheel (d=0.010, w=0.008) on an axle pin captured
  in the cheeks. CONTINUOUS joint about X (free spinning to strike the flint).
- fuel_lever: dark-grey plastic lever; rear curved thumb pad, pivot boss/pin
  inside the hood, forward fork straddling the valve stem. REVOLUTE joint,
  0 to -0.21 rad: pressing the pad DOWN lifts the valve fork UP.
- flip_top_cap: chrome shell cap with a short skirt wrapping the hood upper
  edge, hinge barrel at the rear, grip ridges on the front lip. REVOLUTE
  joint about X at the hinge pin, 0 to ~2.1 rad (120°): positive q lifts the
  front edge upward to expose the flame nozzle.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- tessellation (mm-scale object: default 1 mm tolerance is far too coarse)
TOL = 5.0e-5
ANG = 0.25

# ---- key dimensions (meters) ----
BODY_LEN = 0.0250  # plan long axis (Y)
BODY_DEP = 0.0120  # plan short axis (X)
BODY_H = 0.0640  # top rim of the blue reservoir
WALL = 0.0012  # reservoir wall thickness at the open top
CAVITY_DEPTH = 0.0040  # visible hollow below the rim

DECK_Z0 = 0.0595  # grey valve deck (slightly embedded in cavity floor)
DECK_Z1 = 0.0610

HOOD_Z0 = 0.0630  # chrome hood band wraps the upper ~0.015 m
HOOD_Z1 = 0.0780
HOOD_REAR_CUT_Z = 0.0710  # rear band cut down so the lever pad is exposed

WHEEL_R = 0.0050
WHEEL_W = 0.0080
WHEEL_Y = -0.0030
WHEEL_Z = 0.0755  # axle height -> wheel rim tops out at 0.0805 (~0.08 m tall)

CHEEK_X0 = 0.0058  # inner face of each cheek plate
CHEEK_X1 = 0.0068  # outer face (proud of the hood band)

NOZZLE_Y = 0.0050
NOZZLE_R = 0.0015
NOZZLE_TOP = 0.0745

LEVER_PIVOT = (0.0, -0.0092, 0.0712)
LEVER_PRESS = -0.21  # rad, pad pressed down ~12 degrees

# ---- flip-top cap (variant) ----
HOOD_OUTER_X = BODY_DEP + 0.0012  # hood band outer short axis (0.0132)
CAP_CLEARANCE = 0.0005            # radial clearance between cap inner and hood outer
CAP_WALL = 0.0010                 # cap shell wall thickness
CAP_TOP_T = 0.0012                # cap top plate thickness
CAP_SKIRT = 0.004                 # skirt drop below the top plate
CAP_LENGTH = 0.015                # cap Y extent from hinge forward
CAP_X = HOOD_OUTER_X + 2 * (CAP_CLEARANCE + CAP_WALL)  # cap outer width (~0.0162)
CAP_OPEN_ANGLE = 2.1              # radians (~120°)
# Hinge between the spark wheel (rear) and nozzle (front):
# wheel front edge ~ y = -0.003 + 0.005 = 0.002; nozzle center y = 0.005
HINGE_Y = 0.003                   # just in front of the wheel
HINGE_Z = HOOD_Z1 + 0.0005       # barrel centre 0.5 mm above the hood top
# In cap-local frame (origin at hinge), the body hangs below the barrel:
#   top plate: z from -0.001 to -0.001 + CAP_TOP_T
#   skirt:     z from -0.001 - CAP_SKIRT to -0.001
CAP_BODY_Z_TOP = -0.001           # top plate lower face in cap-local
CAP_BODY_Z_BOT = CAP_BODY_Z_TOP - CAP_SKIRT  # skirt lower face


def _reservoir_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").slot2D(BODY_LEN, BODY_DEP, 90).extrude(BODY_H)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - CAVITY_DEPTH)
        .slot2D(BODY_LEN - 2 * WALL, BODY_DEP - 2 * WALL, 90)
        .extrude(CAVITY_DEPTH + 0.002)
    )
    return outer.cut(cavity)


def _valve_deck() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=DECK_Z0)
        .slot2D(BODY_LEN - 2 * WALL - 0.0004, BODY_DEP - 2 * WALL - 0.0004, 90)
        .extrude(DECK_Z1 - DECK_Z0)
    )


def _hood_band() -> cq.Workplane:
    band = (
        cq.Workplane("XY")
        .workplane(offset=HOOD_Z0)
        .slot2D(BODY_LEN + 0.0012, BODY_DEP + 0.0012, 90)
        .extrude(HOOD_Z1 - HOOD_Z0)
        .cut(
            cq.Workplane("XY")
            .workplane(offset=HOOD_Z0 - 0.0005)
            .slot2D(BODY_LEN - 0.0016, BODY_DEP - 0.0016, 90)
            .extrude(HOOD_Z1 - HOOD_Z0 + 0.0010)
        )
    )
    # Front notch where the flame exits.
    notch = cq.Workplane("XY").box(0.0070, 0.0080, 0.0090).translate((0.0, 0.0120, 0.0745))
    # Rear band cut down so the thumb pad sits exposed above it.
    rear = cq.Workplane("XY").box(0.0200, 0.0070, 0.0080).translate((0.0, -0.0120, 0.0750))
    # Small side vent perforations (through both broad walls).
    vents = (
        cq.Workplane("YZ")
        .pushPoints([(0.0040, 0.0735), (0.0062, 0.0735)])
        .circle(0.0007)
        .extrude(0.0200, both=True)
    )
    return band.cut(notch).cut(rear).cut(vents)


def _hood_cheeks() -> cq.Workplane:
    plate = cq.Workplane("XY").box(CHEEK_X1 - CHEEK_X0, 0.0125, 0.0090)
    plate = plate.edges("|X and >Z").fillet(0.0015)
    xc = 0.5 * (CHEEK_X0 + CHEEK_X1)
    return plate.translate((xc, -0.00375, 0.0745)).union(
        plate.translate((-xc, -0.00375, 0.0745))
    )


def _lever_pad() -> cq.Workplane:
    # Curved thumb tab, authored in the lever-local frame (origin = pivot).
    # Profile in the local YZ plane, extruded across X.
    return (
        cq.Workplane("YZ")
        .polyline([(-0.0002, 0.0010), (-0.0030, 0.0010), (-0.0030, 0.0024)])
        .sagittaArc((-0.0002, 0.0028), -0.0004)
        .close()
        .extrude(0.0036, both=True)
    )


def _lever_fork() -> cq.Workplane:
    # Web dropping from the pivot boss, forward arm, and two fork prongs that
    # straddle the flint tube and the valve stem (lever-local frame).
    web = cq.Workplane("XY").box(0.0040, 0.0026, 0.0032).translate((0.0, 0.0015, -0.0024))
    arm = cq.Workplane("XY").box(0.0052, 0.0040, 0.0010).translate((0.0, 0.0022, -0.0041))
    prong_l = cq.Workplane("XY").box(0.0012, 0.0120, 0.0010).translate((0.0024, 0.0098, -0.0041))
    prong_r = cq.Workplane("XY").box(0.0012, 0.0120, 0.0010).translate((-0.0024, 0.0098, -0.0041))
    return web.union(arm).union(prong_l).union(prong_r)


def _flip_cap_shell() -> cq.Workplane:
    """Flip-top cap shell in the cap-local frame (origin at the hinge barrel centre).

    The cap body hangs below the hinge axis:
    - top plate: z from CAP_BODY_Z_TOP to CAP_BODY_Z_TOP + CAP_TOP_T
    - skirt walls: z from CAP_BODY_Z_BOT to CAP_BODY_Z_TOP
    - extends forward (+Y) from y = 0 to y = CAP_LENGTH

    A rear relief cut removes the skirt near the hinge so the cap clears the
    hood band during opening.
    """
    outer_z = (CAP_BODY_Z_TOP + CAP_TOP_T) - CAP_BODY_Z_BOT  # total outer height
    outer_zc = 0.5 * ((CAP_BODY_Z_TOP + CAP_TOP_T) + CAP_BODY_Z_BOT)

    # Outer shell: rounded-corner box
    outer = (
        cq.Workplane("XY")
        .box(CAP_X, CAP_LENGTH, outer_z)
        .translate((0.0, CAP_LENGTH / 2.0, outer_zc))
    )
    outer = outer.edges("|Z").fillet(0.002)

    # Inner cavity (hollow out the skirt region only, not through the top plate)
    inner_z = CAP_BODY_Z_TOP - CAP_BODY_Z_BOT  # skirt height
    inner_zc = 0.5 * (CAP_BODY_Z_TOP + CAP_BODY_Z_BOT)
    inner = (
        cq.Workplane("XY")
        .box(CAP_X - 2 * CAP_WALL, CAP_LENGTH - 2 * CAP_WALL, inner_z + 0.0002)
        .translate((0.0, CAP_LENGTH / 2.0, inner_zc - 0.0001))
    )

    shell = outer.cut(inner)

    # Rear hinge relief: remove skirt near the hinge for rotation clearance.
    # Cuts from below the skirt up to just below the top plate, over the
    # first ~6 mm of Y from the hinge.
    relief_z = inner_z + 0.001
    relief_zc = 0.5 * (CAP_BODY_Z_BOT + CAP_BODY_Z_TOP)
    relief = (
        cq.Workplane("XY")
        .box(CAP_X + 0.004, 0.006, relief_z)
        .translate((0.0, 0.002, relief_zc))
    )

    return shell.cut(relief)


def _grip_ridge() -> cq.Workplane:
    """Small raised ridge on the cap front lip for finger grip.
    Authored at the origin; caller translates into position.
    """
    return cq.Workplane("XY").box(0.0006, 0.0006, 0.0020)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disposable_flint_pocket_lighter")

    blue = model.material("royal_blue_plastic", rgba=(0.08, 0.13, 0.82, 0.88))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    steel = model.material("knurled_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.27, 0.27, 0.29, 1.0))
    deck_grey = model.material("valve_deck_grey", rgba=(0.55, 0.55, 0.56, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_reservoir_shell(), "reservoir_shell", tolerance=TOL, angular_tolerance=ANG),
        material=blue,
        name="reservoir_shell",
    )
    body.visual(
        mesh_from_cadquery(_valve_deck(), "valve_deck", tolerance=TOL, angular_tolerance=ANG),
        material=deck_grey,
        name="valve_deck",
    )
    # Flame nozzle (valve stem + collar) at the front, rising from the deck.
    body.visual(
        Cylinder(radius=NOZZLE_R, length=NOZZLE_TOP - 0.0600),
        origin=Origin(xyz=(0.0, NOZZLE_Y, 0.5 * (NOZZLE_TOP + 0.0600))),
        material=chrome,
        name="flame_nozzle",
    )
    body.visual(
        Cylinder(radius=0.0026, length=0.0045),
        origin=Origin(xyz=(0.0, NOZZLE_Y, 0.0600 + 0.00225)),
        material=chrome,
        name="nozzle_collar",
    )
    # Flint tube directly under the spark wheel.
    body.visual(
        Cylinder(radius=0.0015, length=0.0098),
        origin=Origin(xyz=(0.0, WHEEL_Y, 0.0600 + 0.0049)),
        material=steel,
        name="flint_tube",
    )
    body.visual(
        mesh_from_cadquery(_hood_band(), "hood_band", tolerance=TOL, angular_tolerance=ANG),
        material=chrome,
        name="hood_band",
    )
    body.visual(
        mesh_from_cadquery(_hood_cheeks(), "hood_cheeks", tolerance=TOL, angular_tolerance=ANG),
        material=chrome,
        name="hood_cheeks",
    )
    # Hinge pin for the flip-top cap, captured through the hood walls.
    body.visual(
        Cylinder(radius=0.0009, length=CAP_X - 0.004),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="cap_hinge_pin",
    )

    # ---------------- spark wheel ----------------
    wheel = model.part("spark_wheel")
    knurl_geom = KnobGeometry(
        2 * WHEEL_R,
        WHEEL_W,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=28, depth=0.0005, helix_angle_deg=25.0),
    )
    wheel.visual(
        mesh_from_geometry(knurl_geom, "spark_wheel_knurl"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),  # knob Z axis -> X axle
        material=steel,
        name="wheel_knurl",
    )
    wheel.visual(
        Cylinder(radius=0.0010, length=0.0136),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wheel_axle",
    )
    model.articulation(
        "body_to_spark_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    # ---------------- fuel lever ----------------
    lever = model.part("fuel_lever")
    lever.visual(
        mesh_from_cadquery(_lever_pad(), "lever_pad", tolerance=TOL, angular_tolerance=ANG),
        material=dark_grey,
        name="lever_pad",
    )
    lever.visual(
        mesh_from_cadquery(_lever_fork(), "lever_fork", tolerance=TOL, angular_tolerance=ANG),
        material=dark_grey,
        name="lever_fork",
    )
    lever.visual(
        Cylinder(radius=0.0016, length=0.0052),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="lever_boss",
    )
    lever.visual(
        Cylinder(radius=0.0009, length=0.0134),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_pin",
    )
    model.articulation(
        "body_to_fuel_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=LEVER_PIVOT),
        # -X axis: q = -0.21 presses the rear thumb pad DOWN and lifts the
        # forward valve fork UP (see-saw about the pivot).
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=LEVER_PRESS, upper=0.0),
    )

    # ---------------- flip-top cap (variant) ----------------
    cap = model.part("flip_top_cap")
    cap.visual(
        mesh_from_cadquery(_flip_cap_shell(), "cap_shell", tolerance=TOL, angular_tolerance=ANG),
        material=chrome,
        name="cap_shell",
    )
    # Hinge barrel wrapping around the body's hinge pin.
    cap.visual(
        Cylinder(radius=0.0014, length=CAP_X - 0.002),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="cap_barrel",
    )
    # Grip ridges on the front lip (for flipping open), emitted via a loop.
    ridge_xs = [-0.005, -0.002, 0.002, 0.005]
    ridge_geom = _grip_ridge()
    for i in range(len(ridge_xs)):
        cap.visual(
            mesh_from_cadquery(ridge_geom, f"cap_grip_{i}", tolerance=TOL, angular_tolerance=ANG),
            origin=Origin(xyz=(ridge_xs[i], CAP_LENGTH + 0.0003, CAP_BODY_Z_TOP - CAP_SKIRT / 2)),
            material=chrome,
            name=f"cap_grip_{i}",
        )

    model.articulation(
        "body_to_flip_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        # Hinge between the spark wheel and the nozzle, at the hood top.
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # +X axis: positive q lifts the front (+Y) edge of the cap upward (+Z).
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=CAP_OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("spark_wheel")
    lever = object_model.get_part("fuel_lever")
    cap = object_model.get_part("flip_top_cap")
    wheel_joint = object_model.get_articulation("body_to_spark_wheel")
    lever_joint = object_model.get_articulation("body_to_fuel_lever")
    cap_joint = object_model.get_articulation("body_to_flip_cap")

    # Intentional captured-pin embeddings (axle/pin pressed through the chrome
    # cheek plates and the hood side walls hidden behind them).
    ctx.allow_overlap(
        wheel, body, elem_a="wheel_axle", elem_b="hood_cheeks",
        reason="spark-wheel axle pin is press-fit into the chrome cheek plates (captured pin)",
    )
    ctx.allow_overlap(
        wheel, body, elem_a="wheel_axle", elem_b="hood_band",
        reason="axle ends pass through the hood side walls hidden behind the cheek plates",
    )
    ctx.allow_overlap(
        lever, body, elem_a="lever_pin", elem_b="hood_cheeks",
        reason="fuel-lever pivot pin is captured in the cheek plates",
    )
    ctx.allow_overlap(
        lever, body, elem_a="lever_pin", elem_b="hood_band",
        reason="lever pivot pin passes through the hood wall behind the cheeks",
    )
    # Cap hinge barrel wraps around the hinge pin (captured pin at the hood top).
    ctx.allow_overlap(
        cap, body, elem_a="cap_barrel", elem_b="cap_hinge_pin",
        reason="flip-cap hinge barrel wraps around the body hinge pin (captured pin)",
    )
    ctx.allow_overlap(
        cap, body, elem_a="cap_barrel", elem_b="hood_band",
        reason="cap hinge barrel passes through the hood wall at the hinge line",
    )

    # --- pins really seated, wheel really between the cheeks ---
    ctx.expect_contact(wheel, body, elem_a="wheel_axle", elem_b="hood_cheeks",
                       name="wheel axle seated in cheek plates")
    ctx.expect_contact(lever, body, elem_a="lever_pin", elem_b="hood_cheeks",
                       name="lever pivot pin seated in cheek plates")
    ctx.expect_within(wheel, body, axes="x", inner_elem="wheel_knurl",
                      outer_elem="hood_cheeks", margin=0.0,
                      name="spark wheel sits between the two cheeks")
    ctx.expect_overlap(wheel, body, axes="yz", elem_a="wheel_knurl",
                       elem_b="hood_cheeks", min_overlap=0.004,
                       name="cheek plates flank the spark wheel")
    # Flint tube rises from the deck to just under the wheel rim.
    ctx.expect_gap(wheel, body, axis="z", positive_elem="wheel_knurl",
                   negative_elem="flint_tube", min_gap=0.0, max_gap=0.002,
                   name="flint tube ends just under the spark wheel")

    knurl = ctx.part_element_world_aabb(wheel, elem="wheel_knurl")
    band = ctx.part_element_world_aabb(body, elem="hood_band")
    pad = ctx.part_element_world_aabb(lever, elem="lever_pad")
    nozzle = ctx.part_element_world_aabb(body, elem="flame_nozzle")
    deck = ctx.part_element_world_aabb(body, elem="valve_deck")
    shell = ctx.part_element_world_aabb(body, elem="reservoir_shell")

    ctx.check(
        "knurled wheel rim protrudes above the chrome hood",
        bool(knurl and band and knurl[1][2] > band[1][2] + 0.0005),
        details=f"knurl={knurl}, band={band}",
    )
    ctx.check(
        "overall height ~0.08 m (wheel rim is the highest point)",
        bool(knurl and 0.078 <= knurl[1][2] <= 0.084),
        details=f"knurl={knurl}",
    )
    ctx.check(
        "body plan ~0.025 x 0.012 m stadium",
        bool(
            shell
            and abs((shell[1][0] - shell[0][0]) - 0.0120) < 0.0010
            and abs((shell[1][1] - shell[0][1]) - 0.0250) < 0.0010
        ),
        details=f"shell={shell}",
    )
    ctx.check(
        "reservoir reads hollow: valve deck recessed below the open rim",
        bool(deck and shell and deck[1][2] <= shell[1][2] - 0.002),
        details=f"deck={deck}, shell={shell}",
    )
    ctx.check(
        "dark-grey thumb pad sits behind the wheel, exposed above the rear hood cut",
        bool(
            pad
            and knurl
            and pad[1][1] < knurl[0][1] - 0.0005  # fully behind the wheel
            and pad[0][2] > 0.0712  # above the cut-down rear band edge (0.0710)
            and pad[1][2] < knurl[1][2]  # below the wheel rim
        ),
        details=f"pad={pad}, knurl={knurl}",
    )
    ctx.check(
        "chrome flame nozzle at the front, inside the hood (under the notch)",
        bool(nozzle and knurl and band and nozzle[0][1] > knurl[1][1] and nozzle[1][2] < band[1][2]),
        details=f"nozzle={nozzle}",
    )

    # --- joint semantics ---
    ctx.check(
        "spark wheel joint is continuous about the left-right (X) axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(wheel_joint.axis[0]) > 0.99
        and abs(wheel_joint.axis[1]) < 1e-6
        and abs(wheel_joint.axis[2]) < 1e-6,
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    lim = lever_joint.motion_limits
    ctx.check(
        "fuel lever is revolute, 0 to -0.21 rad about a parallel horizontal axis",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + 0.21) < 1e-6
        and abs(lim.upper) < 1e-9
        and abs(lever_joint.axis[0]) > 0.99,
        details=f"type={lever_joint.articulation_type}, limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
    )

    # --- decisive poses ---
    fork_rest = ctx.part_element_world_aabb(lever, elem="lever_fork")
    with ctx.pose({lever_joint: LEVER_PRESS}):
        pad_pressed = ctx.part_element_world_aabb(lever, elem="lever_pad")
        fork_pressed = ctx.part_element_world_aabb(lever, elem="lever_fork")
        ctx.check(
            "pressing the lever lowers the thumb pad (~12 deg)",
            bool(pad and pad_pressed and pad_pressed[0][2] < pad[0][2] - 0.0004),
            details=f"rest={pad}, pressed={pad_pressed}",
        )
        ctx.check(
            "pressing the lever lifts the valve fork to open the valve",
            bool(fork_rest and fork_pressed and fork_pressed[1][2] > fork_rest[1][2] + 0.0003),
            details=f"rest={fork_rest}, pressed={fork_pressed}",
        )

    with ctx.pose({wheel_joint: math.pi / 3.0}):
        knurl_spun = ctx.part_element_world_aabb(wheel, elem="wheel_knurl")
        ctx.check(
            "spark wheel spins in place about its axle",
            bool(
                knurl
                and knurl_spun
                and abs(knurl_spun[0][0] - knurl[0][0]) < 1e-4
                and abs(
                    0.5 * (knurl_spun[0][2] + knurl_spun[1][2])
                    - 0.5 * (knurl[0][2] + knurl[1][2])
                )
                < 5e-4
            ),
            details=f"rest={knurl}, spun={knurl_spun}",
        )

    # ===================== flip-top cap tests =====================
    cap_shell_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")
    nozzle_aabb = ctx.part_element_world_aabb(body, elem="flame_nozzle")
    band_aabb = ctx.part_element_world_aabb(body, elem="hood_band")
    hinge_pin_aabb = ctx.part_element_world_aabb(body, elem="cap_hinge_pin")

    # --- cap hinge barrel seated on the hinge pin ---
    ctx.expect_contact(cap, body, elem_a="cap_barrel", elem_b="cap_hinge_pin",
                       name="cap hinge barrel seated on the hinge pin")

    # --- joint semantics ---
    cap_lim = cap_joint.motion_limits
    ctx.check(
        "flip cap is revolute about the left-right (X) hinge axis",
        cap_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(cap_joint.axis[0]) > 0.99
        and abs(cap_joint.axis[1]) < 1e-6
        and abs(cap_joint.axis[2]) < 1e-6,
        details=f"type={cap_joint.articulation_type}, axis={cap_joint.axis}",
    )
    ctx.check(
        "flip cap opens from 0 to ~120 deg (2.1 rad)",
        cap_lim is not None
        and cap_lim.lower is not None
        and cap_lim.upper is not None
        and abs(cap_lim.lower) < 1e-9
        and abs(cap_lim.upper - CAP_OPEN_ANGLE) < 0.05,
        details=f"limits=({cap_lim.lower if cap_lim else None}, {cap_lim.upper if cap_lim else None})",
    )

    # --- closed pose: cap covers the nozzle area ---
    ctx.check(
        "closed cap sits on top of the hood, above the nozzle",
        bool(
            cap_shell_aabb
            and nozzle_aabb
            and cap_shell_aabb[0][2] >= nozzle_aabb[1][2] - 0.002
        ),
        details=f"cap={cap_shell_aabb}, nozzle={nozzle_aabb}",
    )
    ctx.check(
        "closed cap overlaps the hood in XY (covers the top opening)",
        bool(
            cap_shell_aabb
            and band_aabb
            and cap_shell_aabb[0][0] < band_aabb[1][0]
            and cap_shell_aabb[1][0] > band_aabb[0][0]
            and cap_shell_aabb[0][1] < band_aabb[1][1]
            and cap_shell_aabb[1][1] > band_aabb[0][1]
        ),
        details=f"cap={cap_shell_aabb}, band={band_aabb}",
    )

    # --- hinge pin is at the rear of the cap, above the hood ---
    ctx.check(
        "hinge pin is above the hood top at the cap rear edge",
        bool(
            hinge_pin_aabb
            and band_aabb
            and cap_shell_aabb
            and hinge_pin_aabb[0][2] >= band_aabb[1][2] - 0.002
            and hinge_pin_aabb[0][1] <= cap_shell_aabb[0][1] + 0.003
        ),
        details=f"pin={hinge_pin_aabb}, band={band_aabb}, cap={cap_shell_aabb}",
    )

    # --- decisive open pose: cap swings up, exposing the nozzle ---
    with ctx.pose({cap_joint: CAP_OPEN_ANGLE}):
        cap_open = ctx.part_element_world_aabb(cap, elem="cap_shell")
        cap_open_center_z = 0.5 * (cap_open[0][2] + cap_open[1][2]) if cap_open else 0.0
        ctx.check(
            "open cap centre lifts well above the hood top",
            bool(
                cap_open
                and band_aabb
                and cap_open_center_z > band_aabb[1][2] + 0.005
            ),
            details=f"open_cap={cap_open}, band={band_aabb}",
        )
        # When open, the cap top is clearly above the hood.
        ctx.check(
            "open cap top is above the hood (flame exposed)",
            bool(
                cap_open
                and band_aabb
                and cap_open[1][2] > band_aabb[1][2] + 0.010
            ),
            details=f"open_cap={cap_open}, band={band_aabb}",
        )

    # --- spark wheel still exposed above the cap when closed ---
    ctx.check(
        "spark wheel protrudes above the closed cap",
        bool(
            knurl
            and cap_shell_aabb
            and knurl[1][2] > cap_shell_aabb[1][2] - 0.001
        ),
        details=f"knurl={knurl}, cap={cap_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
