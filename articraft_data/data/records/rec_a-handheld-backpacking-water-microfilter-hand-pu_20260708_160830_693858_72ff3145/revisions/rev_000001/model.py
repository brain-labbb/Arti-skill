from __future__ import annotations

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
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Real-world scale constants (meters). This is a backpacking hand-pump water
# microfilter: an olive-green twin-barrel housing (rear pump cylinder fused
# to a shorter front filter-cartridge cylinder), a reciprocating piston
# handle, a twist-off filter cap carrying the clean-water outlet, and an
# intake hose with a pre-filter tip.
# ---------------------------------------------------------------------------

PUMP_R = 0.021
FILTER_R = 0.018
# +X offset of the filter cylinder axis from the pump axis. Chosen so the two
# barrels fuse with a real ~3 mm shared wall while still leaving clearance
# for the (narrower) filter cap above the filter cylinder to clear the
# taller pump cylinder next to it.
FILTER_DX = 0.036
BODY_H = 0.150  # pump cylinder height
FILTER_H = 0.112  # filter cylinder height (shorter than the pump cylinder)
BASE_H = 0.007
BASE_PAD = 0.004

COLLAR_R = PUMP_R + 0.0025
COLLAR_H = 0.016
COLLAR_TOP_Z = BODY_H + COLLAR_H  # 0.166

# The pump cylinder is modeled as a simplified solid mass (the real cylinder
# is hollow internally around the piston). The rod's lower portion is
# intentionally represented as nested/overlapping inside that solid proxy;
# see the scoped `allow_overlap(...)` + retained-insertion checks in
# `run_tests()`.
ROD_R = 0.008
STROKE = 0.085
RETAINED_MIN = 0.020
ROD_BOTTOM_LOCAL_Z = -(STROKE + RETAINED_MIN)  # -0.105
ROD_TOP_LOCAL_Z = 0.028
ROD_LENGTH = ROD_TOP_LOCAL_Z - ROD_BOTTOM_LOCAL_Z  # 0.133
ROD_ORIGIN_Z = (ROD_TOP_LOCAL_Z + ROD_BOTTOM_LOCAL_Z) / 2.0

PADDLE_L = 0.075
PADDLE_W = 0.018
PADDLE_T = 0.013

# The cap is a plain cylindrical twist cap, deliberately narrower than the
# filter cylinder it sits on (a real narrower threaded neck/cap), so it
# never reaches sideways into the neighboring pump cylinder's footprint.
FILTER_CAP_D = 0.024  # 12 mm radius, vs FILTER_R = 18 mm
FILTER_CAP_H = 0.028

PAD_W = 0.018
PAD_T = 0.006
PAD_H = 0.085
PAD_EMBED = 0.0016
PAD_Z = 0.063

RIB_ANGLES_DEG = (210.0, 230.0, 250.0, 270.0, 290.0)
RIB_H = 0.055
RIB_TANGENT_W = 0.006
RIB_THICK = 0.0028
RIB_EMBED = 0.0018
RIB_Z = 0.085

INTAKE_BOSS_R = 0.005
INTAKE_BOSS_LEN = 0.014
INTAKE_BOSS_Z = 0.030

OUTPUT_BOSS_R = 0.0038
OUTPUT_BOSS_LEN = 0.011
OUTPUT_BOSS_Z = 0.015
OUTPUT_BOSS_EMBED = 0.0015

INTAKE_BOSS_EMBED = 0.0015


def _build_housing_pump() -> cq.Workplane:
    """Rear pump-cylinder group: barrel + base pad + top collar."""
    pump_cyl = cq.Workplane("XY").circle(PUMP_R).extrude(BODY_H)
    base_pump = cq.Workplane("XY").circle(PUMP_R + BASE_PAD).extrude(BASE_H)
    collar = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
    )
    return pump_cyl.union(base_pump).union(collar)


def _build_housing_filter() -> cq.Workplane:
    """Front filter-cartridge cylinder group: barrel + base pad."""
    filter_cyl = (
        cq.Workplane("XY")
        .center(FILTER_DX, 0.0)
        .circle(FILTER_R)
        .extrude(FILTER_H)
    )
    base_filter = (
        cq.Workplane("XY")
        .center(FILTER_DX, 0.0)
        .circle(FILTER_R + BASE_PAD)
        .extrude(BASE_H)
    )
    return filter_cyl.union(base_filter)


def _build_grip_pad() -> cq.Workplane:
    pad = (
        cq.Workplane("XY")
        .box(PAD_W, PAD_T, PAD_H)
        .edges("|Z")
        .fillet(0.0025)
    )
    return pad


def _build_paddle() -> cq.Workplane:
    paddle = (
        cq.Workplane("XY")
        .box(PADDLE_L, PADDLE_W, PADDLE_T)
        .edges("|Z")
        .fillet(PADDLE_W * 0.47)
    )
    return paddle


def _rot_z_to_neg_x() -> tuple[float, float, float]:
    # Maps local +Z to world -X.
    return (0.0, -math.pi / 2.0, 0.0)


def _rot_z_to_y() -> tuple[float, float, float]:
    # Maps local +Z to world +Y.
    return (-math.pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backpacking_water_filter_pump")

    model.material("olive_body", rgba=(0.30, 0.36, 0.20, 1.0))
    model.material("olive_cap", rgba=(0.27, 0.33, 0.18, 1.0))
    model.material("rubber_black", rgba=(0.045, 0.045, 0.05, 1.0))
    model.material("fitting_grey", rgba=(0.33, 0.34, 0.35, 1.0))
    model.material("hose_clear", rgba=(0.83, 0.90, 0.88, 0.45))
    model.material("foam_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("clip_blue", rgba=(0.16, 0.38, 0.72, 1.0))

    # -- pump_body (root): twin-cylinder housing, grip pad, ribs, intake hose --
    body = model.part("pump_body")

    housing_pump_mesh = mesh_from_cadquery(_build_housing_pump(), "pump_body_housing_pump")
    body.visual(housing_pump_mesh, material="olive_body", name="housing_pump")

    housing_filter_mesh = mesh_from_cadquery(_build_housing_filter(), "pump_body_housing_filter")
    body.visual(housing_filter_mesh, material="olive_body", name="housing_filter")

    grip_pad_mesh = mesh_from_cadquery(_build_grip_pad(), "pump_body_grip_pad")
    body.visual(
        grip_pad_mesh,
        origin=Origin(xyz=(0.0, PUMP_R + PAD_T / 2.0 - PAD_EMBED, PAD_Z)),
        material="rubber_black",
        name="grip_pad",
    )

    for i, angle_deg in enumerate(RIB_ANGLES_DEG):
        angle = math.radians(angle_deg)
        rib_radius = PUMP_R + RIB_THICK / 2.0 - RIB_EMBED
        body.visual(
            Box((RIB_THICK, RIB_TANGENT_W, RIB_H)),
            origin=Origin(
                xyz=(
                    rib_radius * math.cos(angle),
                    rib_radius * math.sin(angle),
                    RIB_Z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material="olive_body",
            name=f"body_rib_{i}",
        )

    # Intake barb boss near the base of the pump cylinder, pointing -X.
    intake_boss_tip = (
        -(PUMP_R - INTAKE_BOSS_EMBED + INTAKE_BOSS_LEN),
        0.0,
        INTAKE_BOSS_Z,
    )
    body.visual(
        Cylinder(radius=INTAKE_BOSS_R, length=INTAKE_BOSS_LEN),
        origin=Origin(
            xyz=(-(PUMP_R - INTAKE_BOSS_EMBED + INTAKE_BOSS_LEN / 2.0), 0.0, INTAKE_BOSS_Z),
            rpy=_rot_z_to_neg_x(),
        ),
        material="fitting_grey",
        name="intake_barb",
    )

    intake_hose_points = [
        intake_boss_tip,
        (-0.055, 0.012, 0.010),
        (-0.095, 0.028, -0.030),
        (-0.125, 0.045, -0.075),
        (-0.145, 0.055, -0.130),
    ]
    intake_hose_geom = tube_from_spline_points(
        intake_hose_points,
        radius=0.004,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    intake_hose_mesh = mesh_from_geometry(intake_hose_geom, "pump_body_intake_hose")
    body.visual(intake_hose_mesh, material="hose_clear", name="intake_hose")

    intake_tip = intake_hose_points[-1]
    body.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(intake_tip[0], intake_tip[1], intake_tip[2] - 0.012)),
        material="foam_dark",
        name="intake_prefilter",
    )

    # -- pump_handle: reciprocating piston rod + T-bar grip --
    handle = model.part("pump_handle")
    handle.visual(
        Cylinder(radius=ROD_R, length=ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, ROD_ORIGIN_Z)),
        material="olive_body",
        name="handle_rod",
    )
    paddle_mesh = mesh_from_cadquery(_build_paddle(), "pump_handle_paddle")
    handle.visual(
        paddle_mesh,
        origin=Origin(xyz=(0.0, 0.0, ROD_TOP_LOCAL_Z + PADDLE_T / 2.0 - 0.001)),
        material="olive_body",
        name="handle_paddle",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=STROKE, effort=60.0, velocity=0.3),
    )

    # -- filter_cap: twist-off cartridge cap carrying the clean-water outlet --
    cap = model.part("filter_cap")
    cap_knob = KnobGeometry(
        FILTER_CAP_D,
        FILTER_CAP_H,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=12, depth=0.0009),
        center=False,
    )
    cap_mesh = mesh_from_geometry(cap_knob, "filter_cap_knob")
    cap.visual(cap_mesh, material="olive_cap", name="cap_body")

    output_boss_tip = (
        0.0,
        FILTER_CAP_D / 2.0 - OUTPUT_BOSS_EMBED + OUTPUT_BOSS_LEN,
        OUTPUT_BOSS_Z,
    )
    cap.visual(
        Cylinder(radius=OUTPUT_BOSS_R, length=OUTPUT_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, FILTER_CAP_D / 2.0 - OUTPUT_BOSS_EMBED + OUTPUT_BOSS_LEN / 2.0, OUTPUT_BOSS_Z),
            rpy=_rot_z_to_y(),
        ),
        material="fitting_grey",
        name="output_barb",
    )

    output_hose_points = [
        output_boss_tip,
        (0.005, FILTER_CAP_D / 2.0 + 0.05, 0.05),
        (0.010, FILTER_CAP_D / 2.0 + 0.09, 0.10),
        (0.012, FILTER_CAP_D / 2.0 + 0.11, 0.15),
        (0.012, FILTER_CAP_D / 2.0 + 0.12, 0.19),
    ]
    output_hose_geom = tube_from_spline_points(
        output_hose_points,
        radius=0.0035,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    output_hose_mesh = mesh_from_geometry(output_hose_geom, "filter_cap_output_hose")
    cap.visual(output_hose_mesh, material="hose_clear", name="output_hose")

    output_tip = output_hose_points[-1]
    cap.visual(
        Box((0.016, 0.012, 0.022)),
        origin=Origin(xyz=(output_tip[0], output_tip[1] + 0.004, output_tip[2] + 0.006)),
        material="clip_blue",
        name="output_clip",
    )

    model.articulation(
        "body_to_filter_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(FILTER_DX, 0.0, FILTER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.75, effort=3.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("pump_body")
    handle = object_model.get_part("pump_handle")
    cap = object_model.get_part("filter_cap")
    handle_joint = object_model.get_articulation("body_to_handle")
    cap_joint = object_model.get_articulation("body_to_filter_cap")

    housing_pump_visual = body.get_visual("housing_pump")
    housing_filter_visual = body.get_visual("housing_filter")
    rod_visual = handle.get_visual("handle_rod")
    cap_body_visual = cap.get_visual("cap_body")
    intake_barb_visual = body.get_visual("intake_barb")
    intake_hose_visual = body.get_visual("intake_hose")
    intake_prefilter_visual = body.get_visual("intake_prefilter")
    output_barb_visual = cap.get_visual("output_barb")
    output_hose_visual = cap.get_visual("output_hose")
    output_clip_visual = cap.get_visual("output_clip")

    # Overall housing scale reads as a real handheld pump filter.
    housing_aabb = ctx.part_element_world_aabb(body, elem=housing_pump_visual)
    ctx.check(
        "pump body housing height matches the modeled twin-cylinder shape",
        housing_aabb is not None
        and abs((housing_aabb[1][2] - housing_aabb[0][2]) - COLLAR_TOP_Z) < 0.006,
        details=f"housing_aabb={housing_aabb}, expected_height={COLLAR_TOP_Z}",
    )

    # The filter cap is seated on the filter cylinder rim, not floating or sunk.
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        positive_elem=cap_body_visual,
        negative_elem=housing_filter_visual,
        min_gap=-0.004,
        max_gap=0.004,
        name="filter cap seats on the filter cylinder rim",
    )

    # Hose/fitting connectivity: nothing floats disconnected from its boss.
    ctx.expect_contact(
        body,
        body,
        elem_a=intake_barb_visual,
        elem_b=intake_hose_visual,
        contact_tol=0.002,
        name="intake hose connects to the intake barb",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=intake_hose_visual,
        elem_b=intake_prefilter_visual,
        contact_tol=0.002,
        name="intake hose connects to the pre-filter tip",
    )
    ctx.expect_contact(
        cap,
        cap,
        elem_a=output_barb_visual,
        elem_b=output_hose_visual,
        contact_tol=0.002,
        name="output hose connects to the outlet barb",
    )
    ctx.expect_contact(
        cap,
        cap,
        elem_a=output_hose_visual,
        elem_b=output_clip_visual,
        contact_tol=0.002,
        name="output hose connects to the outlet clip",
    )

    # Primary mechanism: the piston handle reciprocates a full stroke. The
    # pump cylinder is authored as a simplified solid mass standing in for
    # the real hollow cylinder, so the rod's lower (hidden) portion is
    # intentionally nested/overlapping inside that solid proxy.
    ctx.allow_overlap(
        body,
        handle,
        elem_a=housing_pump_visual,
        elem_b=rod_visual,
        reason=(
            "The pump cylinder is modeled as a solid proxy standing in for its "
            "real hollow interior; the piston rod's lower portion is represented "
            "as sliding inside that solid mass rather than a modeled bore."
        ),
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem=rod_visual,
        outer_elem=housing_pump_visual,
        margin=0.002,
        name="piston rod stays centered within the pump cylinder",
    )

    with ctx.pose({handle_joint: 0.0}):
        p_low = ctx.part_world_position(handle)
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a=rod_visual,
            elem_b=housing_pump_visual,
            min_overlap=0.100,
            name="rod retains insertion in the pump cylinder at full retraction",
        )

    with ctx.pose({handle_joint: STROKE}):
        p_high = ctx.part_world_position(handle)
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a=rod_visual,
            elem_b=housing_pump_visual,
            min_overlap=0.018,
            name="rod retains insertion in the pump cylinder at full extension",
        )

    ctx.check(
        "pump handle rises by the full modeled stroke",
        p_low is not None
        and p_high is not None
        and abs((p_high[2] - p_low[2]) - STROKE) < 1e-6,
        details=f"p_low={p_low}, p_high={p_high}, stroke={STROKE}",
    )

    # Secondary mechanism: the filter cap can twist open through its modeled
    # unlock range without leaving its seated footprint.
    cap_limits = cap_joint.motion_limits
    ctx.check(
        "filter cap twist range is a partial unlock turn, not a full spin",
        cap_limits is not None
        and cap_limits.lower == 0.0
        and cap_limits.upper is not None
        and 0.2 < cap_limits.upper < 3.2,
        details=f"cap_limits={cap_limits}",
    )
    with ctx.pose({cap_joint: cap_joint.motion_limits.upper}):
        cap_pos_twisted = ctx.part_world_position(cap)
    cap_pos_rest = ctx.part_world_position(cap)
    ctx.check(
        "filter cap stays centered over the filter cylinder while twisting",
        cap_pos_rest is not None
        and cap_pos_twisted is not None
        and abs(cap_pos_rest[0] - cap_pos_twisted[0]) < 1e-6
        and abs(cap_pos_rest[1] - cap_pos_twisted[1]) < 1e-6,
        details=f"cap_pos_rest={cap_pos_rest}, cap_pos_twisted={cap_pos_twisted}",
    )

    return ctx.report()


object_model = build_object_model()
