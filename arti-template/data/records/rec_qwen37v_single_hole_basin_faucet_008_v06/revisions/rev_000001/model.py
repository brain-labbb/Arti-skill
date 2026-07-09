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
# Single-hole basin faucet variant, ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body leans BACK a few degrees, i.e. its long axis tilts toward -X.
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

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Flow knob dimensions.
KNOB_DIAMETER = 0.046
KNOB_HEIGHT = 0.022
KNOB_MOUNT_S = NECK_S1 + 0.001  # just above the neck top

# Side lever housing boss dimensions.
HOUSING_R = 0.014
HOUSING_LEN = 0.028

# Flow knob rotation limits (0 = off, ~90° = full flow).
FLOW_LIMIT = math.radians(90.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim. Built in spout-local frame whose origin sits on the
    body axis at SPOUT_S; the shank runs along local +X."""
    r_out = 0.015
    shank_x0 = 0.010  # seated ~15 mm inside the body casting
    shank_x1 = 0.035
    bend = 0.028  # bend radius; end heads straight down at (0.063, -0.028)
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.006))
        .circle(0.0148)
        .workplane(offset=-0.010)
        .circle(0.0185)
        .loft()
    )
    spout = tube.union(flare)

    # Tapered bore opening the outlet mouth (real hollow outlet rim).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.006))
        .circle(0.0155)
        .workplane(offset=0.018)
        .circle(0.011)
        .loft()
    )
    return spout.cut(bore)


def _build_lever_housing_shape() -> cq.Workplane:
    """Offset side lever housing: a cylindrical boss projecting from the
    body toward +Y with a rounded end. Built as one single revolved solid.
    Housing local frame: origin at the body surface, boss axis along +Y."""
    # Build as a single revolved profile for guaranteed connectivity.
    # Profile in XY plane (will be revolved around Y axis):
    # collar wider at base, then shaft, then rounded end.
    total_len = HOUSING_LEN + 0.008  # include rounded end
    profile = (
        cq.Workplane("XY")
        .moveTo(0.0, -0.002)
        .lineTo(HOUSING_R + 0.002, -0.002)
        .lineTo(HOUSING_R + 0.002, 0.003)
        .lineTo(HOUSING_R, 0.003)
        .lineTo(HOUSING_R, HOUSING_LEN)
        .lineTo(HOUSING_R * 0.6, HOUSING_LEN + 0.006)
        .lineTo(0.0, HOUSING_LEN + 0.008)
        .close()
    )
    housing = profile.revolve(360, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    return housing


def _build_knob_shape():
    """Cylindrical flow control knob with subtle grip grooves."""
    knob = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            diameter=KNOB_DIAMETER + 0.004,
            height=0.003,
            flare=0.04,
            chamfer=0.0008,
        ),
        grip=KnobGrip(style="fluted", count=24, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
        center=False,
    )
    return mesh_from_geometry(knob, "flow_knob")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_variant")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck + housing
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

    # Offset side lever housing boss on the +Y side of the body barrel,
    # below the neck/knob region. Placed at the groove station.
    housing_s = (GROOVE_S0 + GROOVE_S1) / 2.0
    housing_world = _axis_point(housing_s)
    body.visual(
        mesh_from_cadquery(_build_lever_housing_shape(), "lever_housing", tolerance=0.0003),
        origin=Origin(
            xyz=(housing_world[0], housing_world[1] + BODY_R - 0.002, housing_world[2]),
            rpy=(0.0, -TILT, 0.0),
        ),
        material="chrome",
        name="lever_housing",
    )

    # ---------------- spout (fixed): swept hollow tube + flared outlet -----
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
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

    # ---------------- flow knob (revolute on top of the neck) --------------
    knob = model.part("flow_knob")
    knob.visual(
        _build_knob_shape(),
        material="chrome_brushed",
        name="knob_body",
    )

    # Joint frame on the body axis at the neck top; local +z runs up the
    # tilted axis. Axis (0,0,1) means positive q rotates the knob CCW
    # when viewed from above (opening flow).
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=_axis_point(NECK_S1), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=FLOW_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    knob = object_model.get_part("flow_knob")
    knob_joint = object_model.get_articulation("knob_rotate")

    # Intentional seated insertion (spout shank into solid body proxy).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
    )
    # Knob skirt wraps around the neck top (real knob seated on shaft).
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_body",
        elem_b="body_neck",
        reason="Flow knob skirt intentionally seats over the neck top like a real rotary knob on its shaft bore.",
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

    # ---- lever housing: offset to one side, connected to the body --------
    housing_aabb = ctx.part_element_world_aabb(body, elem="lever_housing")
    body_barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
    ctx.check(
        "side lever housing extends well beyond the body on the +Y side",
        housing_aabb is not None
        and body_barrel_aabb is not None
        and housing_aabb[1][1] > body_barrel_aabb[1][1] + 0.015,
        details=f"housing aabb={housing_aabb}, barrel aabb={body_barrel_aabb}",
    )
    # Housing should overlap with the barrel/groove on Z (vertically seated on body)
    ctx.expect_overlap(
        body,
        body,
        axes="z",
        elem_a="lever_housing",
        elem_b="body_barrel",
        min_overlap=0.005,
        name="lever housing overlaps the body barrel vertically (seated on body)",
    )

    # ---- spout: projects forward from the body and curves down ------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low open outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.008,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- flow knob: sits on top of the neck, wider than neck --------------
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "flow knob is wider than the neck (visible grip diameter)",
        knob_aabb is not None
        and neck_aabb is not None
        and (knob_aabb[1][1] - knob_aabb[0][1]) > (neck_aabb[1][1] - neck_aabb[0][1]) + 0.002,
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "overall faucet height is about 0.13 m",
        knob_aabb is not None and 0.115 <= knob_aabb[1][2] <= 0.140,
        details=f"knob aabb={knob_aabb}",
    )

    # Knob should overlap the neck on Z (knob seated on top of the neck bore)
    ctx.expect_overlap(
        knob,
        body,
        axes="z",
        elem_a="knob_body",
        elem_b="body_neck",
        min_overlap=0.002,
        name="flow knob skirt seats over the neck top (seated mount)",
    )

    # ---- articulation: knob rotates for flow control ----------------------
    jl = knob_joint.motion_limits
    ctx.check(
        "knob rotation limits are 0 to 90 degrees",
        jl is not None
        and jl.lower is not None
        and jl.upper is not None
        and abs(jl.lower) < 1e-9
        and abs(jl.upper - FLOW_LIMIT) < 1e-6,
        details=f"limits={jl}",
    )
    ctx.check(
        "knob joint is revolute (non-fixed)",
        knob_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={knob_joint.articulation_type}",
    )

    # ---- decisive pose: rotating the knob moves the indicator mark --------
    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: FLOW_LIMIT}):
        knob_turned = ctx.part_world_position(knob)
    # Knob center should stay roughly in place (revolute, not prismatic)
    ctx.check(
        "knob stays in place when rotated (revolute, no translation)",
        knob_rest is not None
        and knob_turned is not None
        and abs(knob_rest[2] - knob_turned[2]) < 0.002
        and abs(knob_rest[0] - knob_turned[0]) < 0.002,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    return ctx.report()


object_model = build_object_model()
