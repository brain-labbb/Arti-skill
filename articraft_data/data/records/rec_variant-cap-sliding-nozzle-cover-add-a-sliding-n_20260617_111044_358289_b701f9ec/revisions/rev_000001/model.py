from __future__ import annotations

"""Classic disposable flint pocket lighter (BIC style), standing upright.

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
  rear edge for the lever) and two chrome cheek plates flanking the wheel.
- spark_wheel: knurled steel wheel (d=0.010, w=0.008) on an axle pin captured
  in the cheeks. CONTINUOUS joint about X (free spinning to strike the flint).
- fuel_lever: dark-grey plastic lever; rear curved thumb pad, pivot boss/pin
  inside the hood, forward fork straddling the valve stem. REVOLUTE joint,
  0 to -0.21 rad: pressing the pad DOWN lifts the valve fork UP.
- nozzle_cover: chrome sliding nozzle cover that shields/exposes the flame
  nozzle. PRISMATIC joint along +Y, 0 to 0.010 m travel.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

# ---- nozzle cover (sliding variant) ----
COVER_L = 0.0120  # Y length of the stadium plate
COVER_W = 0.0100  # X width (fits inside the hood cavity with clearance)
COVER_T = 0.0008  # Z thickness

SLIDE_ORIGIN_Y = 0.003  # joint-origin Y so cover at q=0 shields the nozzle
SLIDE_UPPER = 0.010  # max travel (m) — fully exposes the flame nozzle

RAIL_W = 0.0010  # each track rail width (X)
RAIL_L = 0.0140  # track rail length (Y)
RAIL_H = 0.0010  # track rail height (Z), slightly taller than the cover
RAIL_X = 0.0059  # |X| center of each rail, sitting on the hood wall
RAIL_Y = 0.006  # Y center of both rails


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


def _nozzle_cover_plate() -> cq.Workplane:
    """Stadium-shaped sliding cover plate with a raised grip ridge.

    Authored in the cover-local frame: bottom face at Z = 0 (sits on the hood
    top surface when the joint is at q = 0), long axis along Y.
    """
    plate = (
        cq.Workplane("XY")
        .slot2D(COVER_L, COVER_W, 90)  # length along Y, width along X
        .extrude(COVER_T)
    )
    # Raised grip ridge near the front (+Y) end for the user's thumb.
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=COVER_T)
        .rect(COVER_W * 0.45, 0.0018)
        .extrude(0.0005)
    )
    ridge = ridge.edges(">Z").fillet(0.0003)
    return plate.union(ridge.translate((0.0, 0.003, 0.0)))


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
    # Slide-track rails: two thin chrome strips on the hood top, guiding the
    # nozzle cover along +Y.
    body.visual(
        Box((RAIL_W, RAIL_L, RAIL_H)),
        origin=Origin(xyz=(+RAIL_X, RAIL_Y, HOOD_Z1 + RAIL_H / 2.0)),
        material=chrome,
        name="slide_rail_0",
    )
    body.visual(
        Box((RAIL_W, RAIL_L, RAIL_H)),
        origin=Origin(xyz=(-RAIL_X, RAIL_Y, HOOD_Z1 + RAIL_H / 2.0)),
        material=chrome,
        name="slide_rail_1",
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

    # ---------------- nozzle cover (sliding) ----------------
    cover = model.part("nozzle_cover")
    cover.visual(
        mesh_from_cadquery(
            _nozzle_cover_plate(), "nozzle_cover_plate",
            tolerance=TOL, angular_tolerance=ANG,
        ),
        material=chrome,
        name="cover_plate",
    )
    model.articulation(
        "body_to_nozzle_cover",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cover,
        # Joint origin on the hood top surface; at q=0 the child frame
        # coincides here and the cover shields the flame nozzle.
        origin=Origin(xyz=(0.0, SLIDE_ORIGIN_Y, HOOD_Z1)),
        axis=(0.0, 1.0, 0.0),  # slides along +Y to expose the nozzle
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.5, lower=0.0, upper=SLIDE_UPPER,
        ),
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

    # ---------------- nozzle cover (sliding) ----------------
    cover = object_model.get_part("nozzle_cover")
    cover_joint = object_model.get_articulation("body_to_nozzle_cover")

    # Intentional overlap: cover plate bottom sits flush on the hood top
    # surface (seated sliding contact).
    ctx.allow_overlap(
        cover, body, elem_a="cover_plate", elem_b="hood_band",
        reason="nozzle cover plate sits flush on the hood top ring (sliding contact surface)",
    )

    # --- joint semantics ---
    clim = cover_joint.motion_limits
    ctx.check(
        "nozzle cover is prismatic along +Y with 0 to 10 mm travel",
        cover_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(cover_joint.axis[1] - 1.0) < 1e-6
        and abs(cover_joint.axis[0]) < 1e-6
        and abs(cover_joint.axis[2]) < 1e-6
        and clim is not None
        and abs(clim.lower) < 1e-9
        and abs(clim.upper - SLIDE_UPPER) < 1e-6,
        details=f"type={cover_joint.articulation_type}, axis={cover_joint.axis}, "
                f"limits=({clim.lower if clim else None}, {clim.upper if clim else None})",
    )

    # --- cover shields nozzle at rest ---
    cover_rest = ctx.part_element_world_aabb(cover, elem="cover_plate")
    ctx.check(
        "cover shields the flame nozzle at rest (covers nozzle Y extent)",
        bool(
            cover_rest and nozzle
            and cover_rest[0][1] <= nozzle[0][1] + 0.0005
            and cover_rest[1][1] >= nozzle[1][1] - 0.0005
        ),
        details=f"cover_rest={cover_rest}, nozzle={nozzle}",
    )
    ctx.check(
        "cover sits above the nozzle (on the hood top)",
        bool(cover_rest and nozzle and cover_rest[0][2] >= nozzle[1][2] - 0.002),
        details=f"cover_rest={cover_rest}, nozzle={nozzle}",
    )

    # --- cover is guided between the slide rails ---
    rail_0 = ctx.part_element_world_aabb(body, elem="slide_rail_0")
    rail_1 = ctx.part_element_world_aabb(body, elem="slide_rail_1")
    ctx.check(
        "cover plate fits between the slide rails (X containment)",
        bool(
            cover_rest and rail_0 and rail_1
            and cover_rest[0][0] > rail_1[0][0] - 0.0005
            and cover_rest[1][0] < rail_0[1][0] + 0.0005
        ),
        details=f"cover_x=({cover_rest[0][0]:.4f},{cover_rest[1][0]:.4f}), "
                f"rail_0_x=({rail_0[0][0]:.4f},{rail_0[1][0]:.4f}), "
                f"rail_1_x=({rail_1[0][0]:.4f},{rail_1[1][0]:.4f})",
    )

    # --- decisive slide pose: cover moves in +Y and exposes the nozzle ---
    cover_rest_y = ctx.part_world_position(cover)
    with ctx.pose({cover_joint: SLIDE_UPPER}):
        cover_open = ctx.part_element_world_aabb(cover, elem="cover_plate")
        cover_open_y = ctx.part_world_position(cover)
        ctx.check(
            "sliding the cover forward (+Y) exposes the flame nozzle",
            bool(
                cover_open and nozzle
                and cover_open[0][1] >= nozzle[1][1] - 0.001
            ),
            details=f"cover_open={cover_open}, nozzle={nozzle}",
        )
        ctx.check(
            "cover translates in +Y by the full travel distance",
            bool(
                cover_rest_y and cover_open_y
                and abs((cover_open_y[1] - cover_rest_y[1]) - SLIDE_UPPER) < 1e-4
            ),
            details=f"rest={cover_rest_y}, open={cover_open_y}",
        )
        ctx.check(
            "cover stays at the same height while sliding (no Z drift)",
            bool(
                cover_rest and cover_open
                and abs(cover_open[0][2] - cover_rest[0][2]) < 1e-4
            ),
            details=f"rest_z={cover_rest[0][2] if cover_rest else None}, "
                    f"open_z={cover_open[0][2] if cover_open else None}",
        )

    return ctx.report()


object_model = build_object_model()
