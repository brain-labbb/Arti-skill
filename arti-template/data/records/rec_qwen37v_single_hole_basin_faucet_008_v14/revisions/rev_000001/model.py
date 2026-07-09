from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
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
# Single-hole basin faucet with waterfall spout and rotary flow knob.
# ~0.13 m tall, mirror chrome. Variant 14 of the pillar tap family.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees (tilted long axis toward -X).
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Thin recessed separation groove ring around the upper third.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Cartridge cap seam: thin ring just below the knob seat.
SEAM_R = 0.024
SEAM_S0 = 0.100
SEAM_S1 = 0.103

# Flow knob (cylindrical, sits on top of the body).
KNOB_JOINT_S = NECK_S1  # joint origin at the top of the neck
KNOB_DIAMETER = 0.048
KNOB_HEIGHT = 0.018

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Knob rotation limits: -90 to +90 degrees for on/off flow control.
KNOB_TURN_LIMIT = math.radians(90.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_waterfall_spout() -> cq.Workplane:
    """Waterfall-style spout: cylindrical shank that transitions to a wide
    flat outlet with a rounded lip. Built in spout-local frame with origin
    on the body axis at SPOUT_S; shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated inside the body casting
    shank_x1 = 0.035
    bend = 0.028
    end_x = shank_x1 + bend
    end_z = -bend

    # Main tube: straight shank then curved bend downward.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Waterfall lip: wide flat rounded outlet at the end of the bend.
    # Instead of a flared conical rim, create a broad flattened torus-like
    # lip that water would cascade over.
    lip_width = 0.032  # wider than tube for waterfall effect
    lip_thickness = 0.006
    lip_depth = 0.020

    # Build the waterfall lip as a rounded rectangular pad at the spout end.
    lip = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z))
        .rect(lip_depth, lip_width)
        .extrude(-lip_thickness)
    )
    # Fillet edges for the rounded waterfall appearance.
    lip = lip.edges("|Z").fillet(0.004)
    lip = lip.edges("#Z").fillet(0.002)

    spout = tube.union(lip)

    # Hollow bore through the spout for water passage.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.008))
        .circle(0.010)
        .extrude(0.020)
    )
    spout = spout.cut(bore)

    return spout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_waterfall")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("seam_ring", rgba=(0.25, 0.27, 0.30, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck + seam --
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2.0),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2.0),
        material="chrome",
        name="body_neck",
    )
    # Cartridge cap seam: thin dark ring below the knob seat.
    body.visual(
        Cylinder(radius=SEAM_R, length=SEAM_S1 - SEAM_S0),
        origin=_tilted((SEAM_S0 + SEAM_S1) / 2.0),
        material="seam_ring",
        name="cartridge_seam",
    )

    # ---------------- spout (fixed): waterfall-style lip -------------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_waterfall_spout(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- flow knob (revolute): cylindrical with grip grooves --
    flow_knob = model.part("flow_knob")
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            diameter=KNOB_DIAMETER + 0.006,
            height=0.003,
            flare=0.0,
            chamfer=0.0008,
        ),
        grip=KnobGrip(style="fluted", count=24, depth=0.001),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob"),
        material="chrome_brushed",
        name="knob_shell",
    )
    # Knob stem: short shaft that inserts into the neck bore for support.
    knob_stem_r = 0.008
    knob_stem_len = 0.012  # extends below the knob mounting face
    flow_knob.visual(
        Cylinder(radius=knob_stem_r, length=knob_stem_len),
        origin=Origin(xyz=(0.0, 0.0, -knob_stem_len / 2.0)),
        material="chrome",
        name="knob_stem",
    )
    # Knob rotates about the tilted body axis for flow on/off.
    # Joint frame at top of neck; axis along local +Z (tilted body axis).
    # Positive q opens flow (rotate knob counterclockwise when viewed from top).
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flow_knob,
        origin=Origin(xyz=_axis_point(KNOB_JOINT_S), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-KNOB_TURN_LIMIT,
            upper=KNOB_TURN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    flow_knob = object_model.get_part("flow_knob")
    knob_joint = object_model.get_articulation("knob_rotate")

    # Intentional seated insertion: spout shank into body barrel.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated inside the solid body casting.",
    )
    # Knob stem inserts into the neck bore for rotary support.
    ctx.allow_overlap(
        flow_knob,
        body,
        elem_a="knob_stem",
        elem_b="body_neck",
        reason="Knob stem shaft inserts into the neck bore as a rotary cartridge mount.",
    )
    # Knob skirt wraps slightly over the neck top for a seated appearance.
    ctx.allow_overlap(
        flow_knob,
        body,
        elem_a="knob_shell",
        elem_b="body_neck",
        reason="Knob skirt sits flush against the neck top ring as a seated trim interface.",
    )
    # Knob stem passes through the cartridge seam ring region.
    ctx.allow_overlap(
        flow_knob,
        body,
        elem_a="knob_stem",
        elem_b="cartridge_seam",
        reason="Knob stem shaft passes through the cartridge cap seam ring as part of the rotary cartridge mount.",
    )

    # ---- hero geometry: flange seated on deck, body leaning back ----------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind the flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # ---- spout: waterfall lip projects forward and curves down ------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.055
        and spout_aabb[0][2] < 0.030
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )
    # Waterfall lip is wider (Y extent) than the tube diameter.
    ctx.check(
        "waterfall lip is wider than the tube diameter",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.028,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- cartridge cap seam exists on the body below the knob -------------
    seam_aabb = ctx.part_element_world_aabb(body, elem="cartridge_seam")
    ctx.check(
        "cartridge seam ring is present on the body",
        seam_aabb is not None and seam_aabb[1][2] > 0.09,
        details=f"seam aabb={seam_aabb}",
    )

    # ---- flow knob: mounted on top, extends above the neck ----------------
    knob_aabb = ctx.part_world_aabb(flow_knob)
    ctx.check(
        "flow knob extends above the neck top",
        knob_aabb is not None
        and neck_aabb is not None
        and knob_aabb[1][2] > neck_aabb[1][2] + 0.010,
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "flow knob is wider than the stepped-in neck",
        knob_aabb is not None
        and neck_aabb is not None
        and (knob_aabb[1][1] - knob_aabb[0][1]) > (neck_aabb[1][1] - neck_aabb[0][1]) + 0.008,
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )

    # Proof checks for knob stem insertion into the neck bore.
    ctx.expect_overlap(
        flow_knob,
        body,
        axes="z",
        elem_a="knob_stem",
        elem_b="body_neck",
        min_overlap=0.005,
        name="knob stem retained inside the neck bore",
    )
    ctx.expect_within(
        flow_knob,
        body,
        axes="xy",
        inner_elem="knob_stem",
        outer_elem="body_neck",
        margin=0.001,
        name="knob stem stays centered in the neck",
    )

    # ---- overall height about 0.13 m -------------------------------------
    ctx.check(
        "overall faucet height is about 0.12-0.14 m",
        knob_aabb is not None and 0.115 <= knob_aabb[1][2] <= 0.140,
        details=f"knob aabb={knob_aabb}",
    )

    # ---- articulation: knob rotation limits are +/-90 degrees ------------
    kl = knob_joint.motion_limits
    ctx.check(
        "knob rotation limits are -90 to +90 degrees",
        kl is not None
        and kl.lower is not None
        and kl.upper is not None
        and abs(kl.lower + KNOB_TURN_LIMIT) < 1e-6
        and abs(kl.upper - KNOB_TURN_LIMIT) < 1e-6,
        details=f"limits={kl}",
    )

    # ---- decisive pose: rotating the knob swings the indicator ------------
    knob_rest_center = ctx.part_world_position(flow_knob)
    with ctx.pose({knob_joint: KNOB_TURN_LIMIT}):
        knob_turned_center = ctx.part_world_position(flow_knob)
    # The knob center position should remain approximately the same since
    # it rotates about its own axis (only Y may shift slightly due to tilt).
    ctx.check(
        "knob rotation keeps the knob on the body axis",
        knob_rest_center is not None
        and knob_turned_center is not None
        and abs(knob_rest_center[2] - knob_turned_center[2]) < 0.003
        and abs(knob_rest_center[0] - knob_turned_center[0]) < 0.003,
        details=f"rest={knob_rest_center}, turned={knob_turned_center}",
    )

    # Knob articulation is REVOLUTE (not FIXED), proving the flow knob rotates.
    ctx.check(
        "flow knob articulation is revolute with nonzero range",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and kl is not None
        and kl.lower is not None
        and kl.upper is not None
        and (kl.upper - kl.lower) > math.radians(90.0),
        details=f"type={knob_joint.articulation_type}, limits={kl}",
    )

    # ---- groove ring present on the body ----------------------------------
    groove_aabb = ctx.part_element_world_aabb(body, elem="groove_ring")
    ctx.check(
        "separation groove ring is present on the body upper third",
        groove_aabb is not None and groove_aabb[0][2] > 0.06,
        details=f"groove aabb={groove_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
