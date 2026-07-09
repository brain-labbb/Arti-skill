from __future__ import annotations

"""Single-hole basin faucet variant: tapered conical body with forward beak spout,
cylindrical flow knob with grip grooves on top, and a thin cartridge cap seam ring
below the knob. Polished chrome finish.

Layout (meters, +Z up, ground at z=0, spout beak extends along +X):
- Round escutcheon base plate sits on the counter.
- A tapered conical column rises from the base (wider at bottom, narrower at top).
- A small forward beak spout protrudes from the upper-front of the cone with a
  dark round outlet at its tip.
- A thin cartridge cap seam ring marks the transition above the cone.
- A short chrome stem post rises from the cap.
- A cylindrical flow knob with fluted grip grooves sits on the stem and rotates
  about the vertical axis (quarter-turn flow control, 0..90 deg).
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
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
# Base escutcheon
BASE_DIAMETER = 0.058
BASE_H = 0.006
BASE_R = BASE_DIAMETER / 2.0

# Tapered conical body
CONE_BASE_R = 0.026
CONE_TOP_R = 0.016
CONE_H = 0.155
CONE_BOT_Z = BASE_H  # 0.006
CONE_TOP_Z = CONE_BOT_Z + CONE_H  # 0.161

# Forward beak spout
BEAK_ROOT_R = 0.009  # radius at the cone surface
BEAK_TIP_R = 0.006  # radius at the spout tip
BEAK_LENGTH = 0.055  # horizontal forward reach
BEAK_CENTER_Z = CONE_TOP_Z - 0.020  # beak axis height: 0.141
BEAK_ROOT_X = CONE_TOP_R  # starts at the cone top radius
BEAK_TIP_X = BEAK_ROOT_X + BEAK_LENGTH  # 0.071

# Outlet at beak tip
OUTLET_R = 0.0045

# Cartridge cap seam ring (thin chrome ring)
CAP_R = 0.019
CAP_H = 0.004
CAP_BOT_Z = CONE_TOP_Z  # 0.161
CAP_TOP_Z = CAP_BOT_Z + CAP_H  # 0.165

# Stem post
STEM_R = 0.005
STEM_H = 0.012
STEM_BOT_Z = CAP_TOP_Z  # 0.165
STEM_TOP_Z = STEM_BOT_Z + STEM_H  # 0.177

# Flow knob
KNOB_DIAMETER = 0.032
KNOB_HEIGHT = 0.022
KNOB_BOT_Z = STEM_TOP_Z  # 0.177
KNOB_TOP_Z = KNOB_BOT_Z + KNOB_HEIGHT  # 0.199
TOTAL_HEIGHT = KNOB_TOP_Z

# Knob rotation range (quarter turn for flow)
KNOB_RANGE = math.radians(90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    cap_chrome = model.material("cap_chrome", rgba=(0.78, 0.80, 0.84, 1.0))

    # -------------------------------------------------------------------
    # Fixed body: base plate, conical column, beak spout, outlet, cap ring, stem
    # -------------------------------------------------------------------
    body = model.part("faucet_body")

    # Round base escutcheon plate
    body.visual(
        Cylinder(radius=BASE_R, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Tapered conical column (CadQuery loft between two circles)
    cone_solid = (
        cq.Workplane("XY")
        .workplane(offset=CONE_BOT_Z)
        .circle(CONE_BASE_R)
        .workplane(offset=CONE_H)
        .circle(CONE_TOP_R)
        .loft()
    )
    body.visual(
        mesh_from_cadquery(cone_solid, "conical_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="conical_body",
    )

    # Forward beak spout - tapered cylinder angled slightly downward
    # Build as a CadQuery tapered cylinder along X axis
    beak = (
        cq.Workplane("YZ")
        .workplane(offset=BEAK_ROOT_X)
        .circle(BEAK_ROOT_R)
        .workplane(offset=BEAK_LENGTH)
        .circle(BEAK_TIP_R)
        .loft()
    )
    body.visual(
        mesh_from_cadquery(beak, "beak_spout"),
        origin=Origin(xyz=(0.0, 0.0, BEAK_CENTER_Z)),
        material=chrome,
        name="beak_spout",
    )

    # Dark outlet disc at the beak tip
    body.visual(
        Cylinder(radius=OUTLET_R, length=0.003),
        origin=Origin(
            xyz=(BEAK_TIP_X, 0.0, BEAK_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark,
        name="outlet_disc",
    )

    # Cartridge cap seam ring (thin solid disk between cone top and stem)
    cap_ring = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_H)
    )
    body.visual(
        mesh_from_cadquery(cap_ring, "cartridge_cap"),
        origin=Origin(xyz=(0.0, 0.0, CAP_BOT_Z)),
        material=cap_chrome,
        name="cartridge_cap",
    )

    # Stem post
    body.visual(
        Cylinder(radius=STEM_R, length=STEM_H),
        origin=Origin(xyz=(0.0, 0.0, STEM_BOT_Z + STEM_H / 2.0)),
        material=chrome,
        name="stem_post",
    )

    # -------------------------------------------------------------------
    # Flow knob: cylindrical knob with fluted grip grooves, rotates on vertical axis
    # Part frame at the knob base center; knob extends upward from z=0 locally.
    # -------------------------------------------------------------------
    knob = model.part("flow_knob")

    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            diameter=KNOB_DIAMETER + 0.002,
            height=0.003,
            flare=0.0,
            chamfer=0.0008,
        ),
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_body"),
        origin=Origin(xyz=(0.0, 0.0, KNOB_HEIGHT / 2.0)),
        material=chrome,
        name="knob_body",
    )
    # Off-center indicator mark on top face for visual flow position
    indicator_r = KNOB_DIAMETER / 2.0 - 0.004  # near edge
    knob.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(xyz=(indicator_r, 0.0, KNOB_HEIGHT - 0.001)),
        material=dark,
        name="flow_indicator",
    )

    # Revolute articulation: knob rotates about vertical axis through stem
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, KNOB_BOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=5.0, lower=0.0, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    knob_turn = object_model.get_articulation("knob_turn")

    # --- Joint plan: revolute about vertical axis, 0..90 deg ---
    ctx.check(
        "knob_turn is revolute 0..90 deg about vertical axis",
        knob_turn.articulation_type == ArticulationType.REVOLUTE
        and abs(knob_turn.axis[0]) < 1e-9
        and abs(knob_turn.axis[1]) < 1e-9
        and abs(abs(knob_turn.axis[2]) - 1.0) < 1e-9
        and knob_turn.motion_limits is not None
        and abs(knob_turn.motion_limits.lower) < 1e-9
        and abs(knob_turn.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={knob_turn.axis}, limits={knob_turn.motion_limits}",
    )

    # --- Grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height is ~0.20 m",
        knob_aabb is not None and 0.18 <= knob_aabb[1][2] <= 0.22,
        details=f"knob_aabb={knob_aabb}",
    )

    # --- Conical body geometry check ---
    conical_aabb = ctx.part_element_world_aabb(body, elem="conical_body")
    base_aabb = ctx.part_element_world_aabb(body, elem="base_plate")
    ctx.check(
        "conical body sits on the base plate and is taller than wide (tapered)",
        conical_aabb is not None
        and base_aabb is not None
        and conical_aabb[0][2] >= base_aabb[0][2] - 0.001
        and (conical_aabb[1][2] - conical_aabb[0][2]) > (conical_aabb[1][0] - conical_aabb[0][0]),
        details=f"conical_aabb={conical_aabb}, base_aabb={base_aabb}",
    )

    # --- Beak spout extends forward ---
    beak_aabb = ctx.part_element_world_aabb(body, elem="beak_spout")
    ctx.check(
        "beak spout extends forward from the body along +X",
        beak_aabb is not None
        and beak_aabb[1][0] > CONE_TOP_R + 0.03
        and beak_aabb[1][0] - beak_aabb[0][0] > 0.04,
        details=f"beak_aabb={beak_aabb}",
    )

    # --- Dark outlet at beak tip ---
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "dark outlet disc is at the beak tip",
        outlet_aabb is not None
        and beak_aabb is not None
        and outlet_aabb[0][0] > beak_aabb[0][0] + BEAK_LENGTH * 0.8,
        details=f"outlet_aabb={outlet_aabb}, beak_aabb={beak_aabb}",
    )

    # --- Cartridge cap seam ring between body and knob ---
    cap_aabb = ctx.part_element_world_aabb(body, elem="cartridge_cap")
    ctx.check(
        "cartridge cap seam ring sits between cone top and knob base",
        cap_aabb is not None
        and cap_aabb[0][2] >= CONE_TOP_Z - 0.001
        and cap_aabb[1][2] <= KNOB_BOT_Z + 0.002,
        details=f"cap_aabb={cap_aabb}",
    )

    # --- Knob is above the body ---
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        min_gap=-0.005,
        max_gap=0.015,
        name="knob sits on top of the body stem",
    )

    # --- Knob has visible grip grooves (fluted detail means width varies) ---
    knob_body_aabb = ctx.part_element_world_aabb(knob, elem="knob_body")
    ctx.check(
        "knob body is cylindrical with diameter ~0.032 m",
        knob_body_aabb is not None
        and 0.028 <= (knob_body_aabb[1][0] - knob_body_aabb[0][0]) <= 0.038
        and 0.028 <= (knob_body_aabb[1][1] - knob_body_aabb[0][1]) <= 0.038,
        details=f"knob_body_aabb={knob_body_aabb}",
    )

    # --- Decisive pose: rotating the knob about vertical axis ---
    rest_indicator_aabb = ctx.part_element_world_aabb(knob, elem="flow_indicator")
    with ctx.pose({knob_turn: KNOB_RANGE}):
        turned_indicator_aabb = ctx.part_element_world_aabb(knob, elem="flow_indicator")
        ctx.check(
            "knob rotation at 90 deg moves the flow indicator to a different angular position",
            rest_indicator_aabb is not None
            and turned_indicator_aabb is not None
            and (
                abs(turned_indicator_aabb[0][1] - rest_indicator_aabb[0][1]) > 0.003
                or abs(turned_indicator_aabb[1][1] - rest_indicator_aabb[1][1]) > 0.003
                or abs(turned_indicator_aabb[0][0] - rest_indicator_aabb[0][0]) > 0.003
            ),
            details=f"rest_indicator={rest_indicator_aabb}, turned_indicator={turned_indicator_aabb}",
        )

    # --- Knob does not collide with the beak spout ---
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        min_gap=0.0,
        positive_elem="knob_body",
        negative_elem="beak_spout",
        name="knob clears the beak spout vertically",
    )

    return ctx.report()


object_model = build_object_model()
