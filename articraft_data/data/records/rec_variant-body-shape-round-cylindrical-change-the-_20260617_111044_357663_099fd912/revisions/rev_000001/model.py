from __future__ import annotations

"""Classic disposable flint pocket lighter — round cylindrical variant.

Frame conventions (object-intrinsic):
- Z is up; the lighter stands on z = 0.
- Y is the front direction (+Y is the flame/notch end).
- X is left-right: the spark-wheel axle runs along X.

Parts:
- body (root): translucent royal-blue round-cylindrical fuel reservoir
  (circular cross-section, d=0.020 m), hollow at the open top (visible wall
  rim), grey valve deck recessed inside, chrome flame nozzle + flint tube
  rising from the deck, polished chrome hood band wrapping the top ~0.015 m
  (front flame notch, side vent perforations, cut-down rear edge for the
  lever) and two chrome cheek plates flanking the wheel.
- spark_wheel: knurled steel wheel (d=0.010, w=0.008) on an axle pin
  captured in the cheeks. CONTINUOUS joint about X (free spinning).
- fuel_lever: dark-grey plastic lever; rear curved thumb pad, pivot
  boss/pin inside the hood, forward fork straddling the valve stem.
  REVOLUTE joint, 0 to -0.21 rad.
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

# ---- tessellation (mm-scale object)
TOL = 5.0e-5
ANG = 0.25

# ---- round body dimensions (meters) ----
BODY_DIA = 0.0200  # circular cross-section diameter
BODY_R = BODY_DIA / 2.0  # 0.010
BODY_H = 0.0640  # top rim of the blue reservoir
WALL = 0.0012  # reservoir wall thickness at the open top
CAVITY_DEPTH = 0.0040  # visible hollow below the rim

DECK_Z0 = 0.0595  # grey valve deck (recessed inside cavity)
DECK_Z1 = 0.0610

HOOD_Z0 = 0.0630  # chrome hood band wraps the upper ~0.015 m
HOOD_Z1 = 0.0780
HOOD_BAND_OR = BODY_R + 0.0006  # outer radius of hood band (~0.0106)
HOOD_BAND_IR = BODY_R - 0.0008  # inner radius of hood band (~0.0092)

WHEEL_R = 0.0050
WHEEL_W = 0.0080
WHEEL_Y = -0.0030
WHEEL_Z = 0.0755  # axle height

CHEEK_X0 = HOOD_BAND_OR - 0.0004  # inner face embedded in hood outer (~0.0102)
CHEEK_X1 = CHEEK_X0 + 0.0010  # outer face proud of hood (~0.0112)
AXLE_LEN = 2.0 * CHEEK_X1 + 0.0010  # axle spans both cheeks (~0.0234)

NOZZLE_Y = 0.0050
NOZZLE_R = 0.0015
NOZZLE_TOP = 0.0745

LEVER_PIVOT = (0.0, -0.0092, 0.0712)
LEVER_PRESS = -0.21  # rad, pad pressed down ~12 degrees


# ---- shared geometry helper ----
def _round_profile(radius: float, z_offset: float = 0.0) -> cq.Workplane:
    """Round body cross-section profile at a given Z offset."""
    return cq.Workplane("XY").workplane(offset=z_offset).circle(radius)


# ---- reservoir (round cylinder with hollow cavity at open top) ----
def _reservoir_shell() -> cq.Workplane:
    outer = _round_profile(BODY_R).extrude(BODY_H)
    cavity = _round_profile(BODY_R - WALL, BODY_H - CAVITY_DEPTH).extrude(
        CAVITY_DEPTH + 0.002
    )
    return outer.cut(cavity)


# ---- valve deck (round disk recessed inside the cavity) ----
def _valve_deck() -> cq.Workplane:
    return _round_profile(BODY_R - WALL - 0.0002, DECK_Z0).extrude(
        DECK_Z1 - DECK_Z0
    )


# ---- chrome hood band wrapping the round body ----
def _hood_band() -> cq.Workplane:
    band = _round_profile(HOOD_BAND_OR, HOOD_Z0).extrude(
        HOOD_Z1 - HOOD_Z0
    ).cut(
        _round_profile(HOOD_BAND_IR, HOOD_Z0 - 0.0005).extrude(
            HOOD_Z1 - HOOD_Z0 + 0.0010
        )
    )
    # Front notch where the flame exits.
    notch = (
        cq.Workplane("XY")
        .box(0.0070, 0.0080, 0.0090)
        .translate((0.0, 0.0120, 0.0745))
    )
    # Rear band cut down so the thumb pad sits exposed above it.
    rear = (
        cq.Workplane("XY")
        .box(0.0240, 0.0070, 0.0080)
        .translate((0.0, -0.0120, 0.0750))
    )
    # Side vent perforations through both walls of the round hood.
    vent_positions = [(0.0040, 0.0735), (0.0062, 0.0735)]
    vents = (
        cq.Workplane("YZ")
        .pushPoints(vent_positions)
        .circle(0.0007)
        .extrude(0.0120, both=True)
    )
    return band.cut(notch).cut(rear).cut(vents)


# ---- chrome cheek plates flanking the spark wheel ----
def _hood_cheeks() -> cq.Workplane:
    plate = cq.Workplane("XY").box(CHEEK_X1 - CHEEK_X0, 0.0125, 0.0090)
    plate = plate.edges("|X and >Z").fillet(0.0015)
    xc = 0.5 * (CHEEK_X0 + CHEEK_X1)
    return plate.translate((xc, -0.00375, 0.0745)).union(
        plate.translate((-xc, -0.00375, 0.0745))
    )


# ---- fuel lever: curved thumb pad (lever-local frame, origin = pivot) ----
def _lever_pad() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline([(-0.0002, 0.0010), (-0.0030, 0.0010), (-0.0030, 0.0024)])
        .sagittaArc((-0.0002, 0.0028), -0.0004)
        .close()
        .extrude(0.0036, both=True)
    )


# ---- fuel lever: forward fork straddling the valve stem ----
def _lever_fork() -> cq.Workplane:
    web = cq.Workplane("XY").box(0.0040, 0.0026, 0.0032).translate((0.0, 0.0015, -0.0024))
    arm = cq.Workplane("XY").box(0.0052, 0.0040, 0.0010).translate((0.0, 0.0022, -0.0041))
    prong_l = cq.Workplane("XY").box(0.0012, 0.0120, 0.0010).translate((0.0024, 0.0098, -0.0041))
    prong_r = cq.Workplane("XY").box(0.0012, 0.0120, 0.0010).translate((-0.0024, 0.0098, -0.0041))
    return web.union(arm).union(prong_l).union(prong_r)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disposable_flint_pocket_lighter_round")

    blue = model.material("royal_blue_plastic", rgba=(0.08, 0.13, 0.82, 0.88))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    steel = model.material("knurled_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.27, 0.27, 0.29, 1.0))
    deck_grey = model.material("valve_deck_grey", rgba=(0.55, 0.55, 0.56, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_reservoir_shell(), "reservoir_shell",
                           tolerance=TOL, angular_tolerance=ANG),
        material=blue,
        name="reservoir_shell",
    )
    body.visual(
        mesh_from_cadquery(_valve_deck(), "valve_deck",
                           tolerance=TOL, angular_tolerance=ANG),
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
        mesh_from_cadquery(_hood_band(), "hood_band",
                           tolerance=TOL, angular_tolerance=ANG),
        material=chrome,
        name="hood_band",
    )
    body.visual(
        mesh_from_cadquery(_hood_cheeks(), "hood_cheeks",
                           tolerance=TOL, angular_tolerance=ANG),
        material=chrome,
        name="hood_cheeks",
    )

    # ---------------- spark wheel ----------------
    wheel = model.part("spark_wheel")
    knurl_geom = KnobGeometry(
        2 * WHEEL_R,
        WHEEL_W,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=28, depth=0.0005,
                       helix_angle_deg=25.0),
    )
    wheel.visual(
        mesh_from_geometry(knurl_geom, "spark_wheel_knurl"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),  # knob Z axis -> X axle
        material=steel,
        name="wheel_knurl",
    )
    wheel.visual(
        Cylinder(radius=0.0010, length=AXLE_LEN),
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
        mesh_from_cadquery(_lever_pad(), "lever_pad",
                           tolerance=TOL, angular_tolerance=ANG),
        material=dark_grey,
        name="lever_pad",
    )
    lever.visual(
        mesh_from_cadquery(_lever_fork(), "lever_fork",
                           tolerance=TOL, angular_tolerance=ANG),
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
        Cylinder(radius=0.0009, length=AXLE_LEN - 0.0010),
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
        motion_limits=MotionLimits(effort=3.0, velocity=2.0,
                                    lower=LEVER_PRESS, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("spark_wheel")
    lever = object_model.get_part("fuel_lever")
    wheel_joint = object_model.get_articulation("body_to_spark_wheel")
    lever_joint = object_model.get_articulation("body_to_fuel_lever")

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

    # --- round cylindrical body claims ---
    ctx.check(
        "round cylindrical body: diameter ~0.020 m",
        bool(
            shell
            and abs((shell[1][0] - shell[0][0]) - BODY_DIA) < 0.0015
            and abs((shell[1][1] - shell[0][1]) - BODY_DIA) < 0.0015
        ),
        details=f"shell={shell}",
    )
    ctx.check(
        "body cross-section is circular (X span ≈ Y span)",
        bool(
            shell
            and abs(
                (shell[1][0] - shell[0][0]) - (shell[1][1] - shell[0][1])
            ) < 0.0015
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
            and pad[0][2] > 0.0712  # above the cut-down rear band edge
            and pad[1][2] < knurl[1][2]  # below the wheel rim
        ),
        details=f"pad={pad}, knurl={knurl}",
    )
    ctx.check(
        "chrome flame nozzle at the front, inside the hood (under the notch)",
        bool(nozzle and knurl and band
             and nozzle[0][1] > knurl[1][1]
             and nozzle[1][2] < band[1][2]),
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
        details=f"type={lever_joint.articulation_type}, "
                f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
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
            bool(fork_rest and fork_pressed
                 and fork_pressed[1][2] > fork_rest[1][2] + 0.0003),
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

    return ctx.report()


object_model = build_object_model()
