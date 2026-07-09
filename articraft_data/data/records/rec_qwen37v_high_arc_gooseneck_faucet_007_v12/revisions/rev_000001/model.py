from __future__ import annotations

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    sweep_profile_along_spline,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant (forked from gloss-black monobloc mixer).
#
# Variant 12 changes:
# - Spout is lower and wider with a flattened oval tube cross-section.
# - A temperature ring rotates around the pedestal.
# - Shallow ribbing on the spray head (circumferential ribs on tip sleeve).
#
# Layout (world frame, deck plane at z = 0):
# - +X is front (the direction the gooseneck arcs over the sink), +Z is up.
# - Chrome base disc on the deck; gloss-black cylindrical column (0.04 m dia)
#   rises on the Z axis.
# - Horizontal cross-cylinder (0.045 m dia, 0.18 m end-to-end along Y) passes
#   through the column at z = 0.085, forming two valve bodies with end caps.
# - From each valve body top a slim pin lever (0.012 m dia, 0.10 m long)
#   points up; each is an independent revolute joint about the valve Y axis.
# - A thin chrome collar ring (z 0.130..0.140) separates the column from the
#   swivel spout.
# - The gooseneck tube is a flattened oval (0.040 m wide × 0.024 m tall) that
#   arcs up and over, apex ~0.34 m, ending in a chrome tip sleeve with shallow
#   circumferential ribs and a downward outlet.
# - A temperature ring (annular body with indicator tab) sits on the pedestal
#   between the base disc and the cross valve; it rotates about the column
#   axis (revolute, ±150°).
# ---------------------------------------------------------------------------

# ── Base + column (same as parent) ─────────────────────────────────────────
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# ── Cross valve cylinder ──────────────────────────────────────────────────
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# ── Pin levers ────────────────────────────────────────────────────────────
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# ── Swivel collar ─────────────────────────────────────────────────────────
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# ── Gooseneck: lower, wider, flattened oval ───────────────────────────────
OVAL_RX = 0.012   # half-extent in the arc-plane direction (0.024 m)
OVAL_RY = 0.020   # half-extent perpendicular to arc plane (0.040 m wide)

SPOUT_PATH = [
    (0.0, 0.0, 0.0),       # base of riser at collar top
    (0.0, 0.0, 0.06),      # mid riser
    (0.0, 0.0, 0.10),      # top of straight riser
    (0.05, 0.0, 0.165),    # arc climbing
    (0.09, 0.0, 0.19),     # apex
    (0.13, 0.0, 0.165),    # arc descending
    (0.18, 0.0, 0.10),     # end of arc
    (0.18, 0.0, 0.06),     # drop leg tip
]

REACH_X = 0.18          # horizontal reach of the spout end
DROP_END = 0.06         # spout-local z of tube tip (world 0.20)

SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028
AERATOR_R = 0.0118
AERATOR_LEN = 0.003

APEX_WORLD_Z = SWIVEL_Z + 0.19 + OVAL_RX   # ≈ 0.342 m

# ── Temperature ring ──────────────────────────────────────────────────────
RING_Z = 0.030            # world z of ring articulation origin
RING_INNER_R = 0.0215     # bore clears the column (0.020)
RING_OUTER_R = 0.034
RING_HEIGHT = 0.010

# ── Spray ribs ────────────────────────────────────────────────────────────
RIB_TUBE = 0.0012
NUM_RIBS = 3

# ── Joint limits ──────────────────────────────────────────────────────────
SWIVEL_LIMIT = math.radians(110.0)
RING_LIMIT = math.radians(150.0)


def _ellipse_profile(rx: float, ry: float, segments: int = 48):
    """Closed elliptical 2-D profile."""
    return [
        (
            rx * math.cos(2.0 * math.pi * i / segments),
            ry * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _gooseneck_mesh():
    """Flattened-oval tube swept along the gooseneck spline."""
    profile = _ellipse_profile(OVAL_RX, OVAL_RY, segments=48)
    return sweep_profile_along_spline(
        SPOUT_PATH,
        profile=profile,
        samples_per_segment=16,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _temp_ring_shape():
    """Annular ring body (washer) with a small radial indicator tab."""
    ring = (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_HEIGHT)
    )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    gunmetal = model.material("brushed_gunmetal", rgba=(0.22, 0.22, 0.25, 1.0))

    # ──────────────────────────────────────────────────────── body column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ──────────────────────────────────────────────────────── gooseneck spout
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_geometry(_gooseneck_mesh(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    # Shallow circumferential ribs on the spray head sleeve
    for i in range(NUM_RIBS):
        rib_z = DROP_END + 0.005 + i * 0.008
        spout.visual(
            mesh_from_geometry(
                TorusGeometry(radius=SLEEVE_R, tube=RIB_TUBE),
                f"spray_rib_{i}",
            ),
            origin=Origin(xyz=(REACH_X, 0.0, rib_z)),
            material=chrome,
            name=f"spray_rib_{i}",
        )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5,
            lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT,
        ),
    )

    # ──────────────────────────────────────────────────────── temperature ring
    temp_ring = model.part("temperature_ring")
    temp_ring.visual(
        mesh_from_cadquery(_temp_ring_shape(), "ring_body"),
        origin=Origin(xyz=(0.0, 0.0, -RING_HEIGHT / 2.0)),
        material=gunmetal,
        name="ring_body",
    )
    # Indicator tab on the ring outer surface (overlaps ring body 1 mm for
    # within-part connectivity)
    temp_ring.visual(
        Box((0.004, 0.008, RING_HEIGHT * 0.7)),
        origin=Origin(xyz=(RING_OUTER_R + 0.001, 0.0, 0.0)),
        material=gunmetal,
        name="ring_indicator",
    )

    model.articulation(
        "ring_rotation",
        ArticulationType.REVOLUTE,
        parent=column,
        child=temp_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0,
            lower=-RING_LIMIT, upper=RING_LIMIT,
        ),
    )

    # ──────────────────────────────────────────────────────── pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0,
                lower=-math.pi / 2.0, upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")
    temp_ring = object_model.get_part("temperature_ring")

    swivel = object_model.get_articulation("spout_swivel")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")
    ring_rot = object_model.get_articulation("ring_rotation")

    # ── Overlap allowances ────────────────────────────────────────────────
    ctx.allow_overlap(
        lever_0, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats a few mm into the valve cylinder.",
    )
    # The temperature ring has a small clearance gap around the column so it
    # can rotate freely; it is retained by its revolute articulation.
    ctx.allow_isolated_part(
        temp_ring,
        reason="Temperature ring rotates around the pedestal with a small "
               "clearance gap; retained by the ring_rotation articulation.",
    )

    # ── Grounding and scale ───────────────────────────────────────────────
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )

    # ── Changed geometry: lower apex, wider reach ─────────────────────────
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex is high-arc but lower than parent (0.30..0.36 m)",
        spout_aabb is not None and 0.30 <= spout_aabb[1][2] <= 0.36,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck reaches wider over the sink (>= 0.17 m in +X)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.17,
        details=f"spout aabb={spout_aabb}",
    )

    # ── Flattened oval tube cross-section ─────────────────────────────────
    tube = ctx.part_element_world_aabb(spout, elem="gooseneck_tube")
    ctx.check(
        "gooseneck tube has flattened oval cross-section (Y extent >= 0.036 m)",
        tube is not None and (tube[1][1] - tube[0][1]) >= 0.036,
        details=f"tube aabb={tube}",
    )
    # The full swept tube AABB includes the arc reach in X, so we verify
    # the oval width from Y extent alone (0.040 m > circular 0.030 m).
    ctx.check(
        "oval tube Y extent (left-right width) exceeds a circular tube",
        tube is not None and (tube[1][1] - tube[0][1]) >= 0.036,
        details=f"tube aabb={tube}",
    )

    # ── Spout seats on collar ─────────────────────────────────────────────
    ctx.expect_contact(
        spout, column,
        elem_a="gooseneck_tube", elem_b="swivel_collar",
        contact_tol=0.002,
        name="gooseneck riser seats on the chrome collar",
    )

    # ── Chrome tip sleeve with outlet ─────────────────────────────────────
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "chrome tip sleeve at the drop leg with a downward outlet",
        sleeve is not None and aerator is not None
        and aerator[0][2] < sleeve[0][2]
        and abs(0.5 * (sleeve[0][0] + sleeve[1][0]) - REACH_X) < 0.005,
        details=f"sleeve={sleeve}, aerator={aerator}",
    )

    # ── Spray head ribbing ────────────────────────────────────────────────
    rib_aabbs = []
    for i in range(NUM_RIBS):
        rib_aabbs.append(
            ctx.part_element_world_aabb(spout, elem=f"spray_rib_{i}")
        )
    ctx.check(
        f"spray head has {NUM_RIBS} shallow circumferential ribs",
        all(r is not None for r in rib_aabbs),
        details=f"ribs={rib_aabbs}",
    )
    if sleeve is not None and all(r is not None for r in rib_aabbs):
        ctx.check(
            "ribs sit along the spray head sleeve region",
            all(
                abs(0.5 * (r[0][0] + r[1][0]) - REACH_X) < 0.01
                for r in rib_aabbs
            )
            and all(
                r[0][2] >= sleeve[0][2] - 0.005
                and r[1][2] <= sleeve[1][2] + 0.015
                for r in rib_aabbs
            ),
            details=f"sleeve={sleeve}, ribs={rib_aabbs}",
        )

    # ── Temperature ring: position and joint ──────────────────────────────
    ring_aabb = ctx.part_world_aabb(temp_ring)
    ctx.check(
        "temperature ring sits on the pedestal between base and cross valve",
        ring_aabb is not None
        and ring_aabb[0][2] >= 0.005
        and ring_aabb[1][2] <= CROSS_Z,
        details=f"ring aabb={ring_aabb}",
    )
    # Proof check: ring and column share the XY footprint (ring wraps column)
    ctx.expect_overlap(
        temp_ring, column,
        axes="xy",
        elem_a="ring_body", elem_b="column_shaft",
        min_overlap=0.030,
        name="temperature ring wraps around the column in XY",
    )
    ctx.check(
        "ring_rotation is revolute about Z with ±150° limits",
        ring_rot.articulation_type == ArticulationType.REVOLUTE
        and tuple(ring_rot.axis) == (0.0, 0.0, 1.0)
        and ring_rot.motion_limits is not None
        and abs(ring_rot.motion_limits.lower + RING_LIMIT) < 1e-6
        and abs(ring_rot.motion_limits.upper - RING_LIMIT) < 1e-6,
    )

    # Ring pose: indicator rotates about the column axis
    rest_indicator = ctx.part_element_world_aabb(temp_ring, elem="ring_indicator")
    with ctx.pose({ring_rot: math.pi / 2.0}):
        rotated_indicator = ctx.part_element_world_aabb(
            temp_ring, elem="ring_indicator"
        )
    ctx.check(
        "temperature ring rotates the indicator around the column",
        rest_indicator is not None and rotated_indicator is not None
        and abs(rotated_indicator[1][1] - rest_indicator[1][1]) > 0.02,
        details=f"rest={rest_indicator}, rotated={rotated_indicator}",
    )

    # ── Spout swivel joint ────────────────────────────────────────────────
    ctx.check(
        "spout swivel is revolute ±110° about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # Swivel pose: outlet sweeps sideways
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.01
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.05,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ── Lever pivot joints ────────────────────────────────────────────────
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0° about the valve Y axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # Lever boss seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        ctx.expect_overlap(
            lever, column,
            axes="z",
            elem_a="lever_boss", elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )

    return ctx.report()


object_model = build_object_model()
