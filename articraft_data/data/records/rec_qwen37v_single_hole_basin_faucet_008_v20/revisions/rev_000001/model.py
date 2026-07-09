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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant, ~0.12 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Tapered conical body with a small forward beak spout, cylindrical flow
# knob on top that rotates for flow control, cartridge cap seam ring.
# ---------------------------------------------------------------------------

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Tapered conical body.
BODY_R_BOTTOM = 0.025  # wider at the base
BODY_R_TOP = 0.017     # narrower at top
BODY_Z0 = FLANGE_H
BODY_Z1 = 0.100
BODY_H = BODY_Z1 - BODY_Z0

# Cartridge cap seam ring — thin recessed groove near the top.
SEAM_Z = 0.088
SEAM_H = 0.003
SEAM_R = BODY_R_BOTTOM - (BODY_R_BOTTOM - BODY_R_TOP) * ((SEAM_Z - BODY_Z0) / BODY_H) - 0.002

# Neck bore above seam for the knob shaft.
NECK_R = 0.012
NECK_Z0 = BODY_Z1
NECK_Z1 = BODY_Z1 + 0.008

# Spout exit: mid-height on the body front.
SPOUT_Z = 0.050
SPOUT_R = 0.010
SPOUT_LEN = 0.045  # forward projection

# Flow knob on top.
KNOB_DIA = 0.032
KNOB_H = 0.022
KNOB_Z = NECK_Z1  # knob sits on top of neck

# Flow rotation: quarter-turn for on/off.
FLOW_LIMIT = math.radians(90.0)


def _body_radius_at(z: float) -> float:
    """Interpolate the conical body radius at height z."""
    t = (z - BODY_Z0) / BODY_H
    return BODY_R_BOTTOM + (BODY_R_TOP - BODY_R_BOTTOM) * t


def _build_tapered_body() -> cq.Workplane:
    """Tapered conical body with slight fillet at the base and top."""
    body = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z0))
        .circle(BODY_R_BOTTOM)
        .workplane(offset=BODY_H)
        .circle(BODY_R_TOP)
        .loft()
    )
    # Soften the bottom edge where it meets the flange.
    body = body.edges("<Z").fillet(0.002)
    body = body.edges(">Z").fillet(0.001)
    return body


def _build_beak_spout() -> cq.Workplane:
    """Short forward beak spout: cylindrical tube projecting from the body
    front at mid-height, curving slightly downward with a flared outlet."""
    r_out = SPOUT_R
    # Spout origin on the body surface at SPOUT_Z, projecting along +X.
    body_r = _body_radius_at(SPOUT_Z)
    x_start = body_r - 0.005  # seated slightly inside the body wall
    x_end = x_start + SPOUT_LEN
    # Smooth downward bend at the end.
    bend_r = 0.012
    x_bend_start = x_end - bend_r
    z_drop = bend_r * 0.6

    path = (
        cq.Workplane("XZ")
        .moveTo(x_start, 0.0)
        .lineTo(x_bend_start, 0.0)
        .tangentArcPoint((bend_r, -z_drop), relative=True)
    )
    tube = (
        cq.Workplane("YZ", origin=(x_start, 0.0, 0.0))
        .circle(r_out)
        .sweep(path)
    )

    # Flared outlet rim at the downward end.
    end_x = x_bend_start + bend_r
    end_z = -z_drop
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.004))
        .circle(r_out - 0.001)
        .workplane(offset=-0.006)
        .circle(r_out + 0.003)
        .loft()
    )
    spout = tube.union(flare)

    # Hollow bore through the outlet.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.004))
        .circle(r_out - 0.002)
        .workplane(offset=0.012)
        .circle(r_out - 0.004)
        .loft()
    )
    spout = spout.cut(bore)

    # Translate to the correct height.
    spout = spout.translate((0.0, 0.0, SPOUT_Z))
    return spout


def _build_seam_ring() -> cq.Workplane:
    """Thin recessed seam ring around the body near the top."""
    r_outer = _body_radius_at(SEAM_Z + SEAM_H / 2.0) + 0.0005
    ring = (
        cq.Workplane("XY", origin=(0.0, 0.0, SEAM_Z))
        .circle(r_outer)
        .circle(r_outer - 0.002)
        .extrude(SEAM_H)
    )
    return ring


def _build_neck_shaft() -> cq.Workplane:
    """Short neck/shaft above the body for the knob to mount on."""
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, NECK_Z0))
        .circle(NECK_R)
        .extrude(NECK_Z1 - NECK_Z0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("seam_groove", rgba=(0.30, 0.32, 0.35, 1.0))

    # ---------------- body (root): flange + conical body + neck ------------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        mesh_from_cadquery(_build_tapered_body(), "tapered_body", tolerance=0.0003),
        material="chrome",
        name="body_cone",
    )
    body.visual(
        mesh_from_cadquery(_build_seam_ring(), "seam_ring", tolerance=0.0003),
        material="seam_groove",
        name="cartridge_seam",
    )
    body.visual(
        mesh_from_cadquery(_build_neck_shaft(), "neck_shaft", tolerance=0.0003),
        material="chrome",
        name="neck_boss",
    )

    # ---------------- spout (fixed): short forward beak --------------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_beak_spout(), "beak_spout", tolerance=0.0003),
        material="chrome",
        name="beak_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---------------- flow knob (revolute): cylindrical knob on top ------
    # Knob with subtle ribbed grip grooves and an indicator line.
    knob_geom = KnobGeometry(
        KNOB_DIA,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=16, depth=0.0006, width=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
        center=False,  # mounting face at z=0
    )

    knob = model.part("flow_knob")
    # Knob part frame is already at KNOB_Z via the articulation origin,
    # so visuals are relative to that frame (z=0 = mounting face).
    knob.visual(
        mesh_from_geometry(knob_geom, "flow_knob"),
        origin=Origin(),
        material="chrome_brushed",
        name="knob_shell",
    )
    # Brushed flat cap on top of the knob for visual detail.
    knob.visual(
        Cylinder(radius=KNOB_DIA / 2.0 - 0.002, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, KNOB_H - 0.001)),
        material="chrome",
        name="knob_cap",
    )
    # Small indicator dot on the knob rim for tracking rotation.
    knob.visual(
        Cylinder(radius=0.002, length=0.003),
        origin=Origin(xyz=(KNOB_DIA / 2.0 - 0.001, 0.0, KNOB_H / 2.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="chrome_dark",
        name="flow_indicator_dot",
    )

    # Revolute joint: knob rotates about Z axis for flow on/off.
    # Positive q opens flow (quarter turn).
    # The knob seats 2 mm over the neck boss (press-fit insertion).
    model.articulation(
        "knob_flow",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, KNOB_Z - 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
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
    flow_joint = object_model.get_articulation("knob_flow")

    # Intentional seated insertion: spout shank inside body wall.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="beak_tube",
        elem_b="body_cone",
        reason="Spout shank is intentionally seated ~5 mm inside the solid conical body wall.",
    )
    # Knob bore sits over the neck boss.
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_shell",
        elem_b="neck_boss",
        reason="Knob bore is press-fit over the neck boss shaft.",
    )

    # ---- hero geometry: tapered conical body on flange --------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    cone_aabb = ctx.part_element_world_aabb(body, elem="body_cone")
    ctx.check(
        "body is tapered (wider at base, narrower at top)",
        cone_aabb is not None
        and (cone_aabb[1][0] - cone_aabb[0][0]) > 0.030
        and (cone_aabb[1][2] - cone_aabb[0][2]) > 0.080,
        details=f"cone aabb={cone_aabb}",
    )

    # ---- cartridge seam ring present near the top of body -----------------
    seam_aabb = ctx.part_element_world_aabb(body, elem="cartridge_seam")
    ctx.check(
        "cartridge cap seam ring is present below the knob",
        seam_aabb is not None
        and seam_aabb[0][2] > 0.080
        and seam_aabb[1][2] < NECK_Z1 + 0.001,
        details=f"seam aabb={seam_aabb}",
    )

    # ---- beak spout: projects forward from the body -----------------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "beak spout projects forward (+X) from the body",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.050
        and spout_aabb[0][0] < 0.030,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "beak spout outlet droops below the spout root",
        spout_aabb is not None
        and spout_aabb[0][2] < SPOUT_Z - 0.003,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- flow knob: sits on top, wider than the neck ----------------------
    knob_aabb = ctx.part_world_aabb(knob)
    neck_aabb = ctx.part_element_world_aabb(body, elem="neck_boss")
    ctx.check(
        "flow knob sits on top of the body and is wider than the neck",
        knob_aabb is not None
        and neck_aabb is not None
        and knob_aabb[1][2] > NECK_Z1
        and (knob_aabb[1][0] - knob_aabb[0][0]) > (neck_aabb[1][0] - neck_aabb[0][0]) + 0.005,
        details=f"knob aabb={knob_aabb}, neck aabb={neck_aabb}",
    )

    ctx.check(
        "overall faucet height is about 0.12 m",
        knob_aabb is not None and 0.115 <= knob_aabb[1][2] <= 0.140,
        details=f"knob aabb={knob_aabb}",
    )

    # ---- articulation: flow knob is revolute with quarter-turn limits -----
    fl = flow_joint.motion_limits
    ctx.check(
        "flow knob joint is revolute with 0 to 90 degree limits",
        fl is not None
        and fl.lower is not None
        and fl.upper is not None
        and abs(fl.lower) < 1e-9
        and abs(fl.upper - FLOW_LIMIT) < 1e-6,
        details=f"limits={fl}",
    )

    # ---- decisive pose: rotating the knob swings the indicator dot --------
    dot_rest = ctx.part_element_world_aabb(knob, elem="flow_indicator_dot")
    with ctx.pose({flow_joint: FLOW_LIMIT}):
        dot_open = ctx.part_element_world_aabb(knob, elem="flow_indicator_dot")
    ctx.check(
        "rotating the flow knob 90 deg swings the indicator dot around the axis",
        dot_rest is not None
        and dot_open is not None
        and abs(dot_rest[0][1] - dot_open[0][1]) > 0.005,
        details=f"rest={dot_rest}, open={dot_open}",
    )

    # Knob stays at the same height when rotated (pure Z-axis revolute).
    rest_pos = ctx.part_world_position(knob)
    with ctx.pose({flow_joint: FLOW_LIMIT}):
        open_pos = ctx.part_world_position(knob)
    ctx.check(
        "knob stays at the same height when rotated (revolute about Z)",
        rest_pos is not None
        and open_pos is not None
        and abs(rest_pos[2] - open_pos[2]) < 0.001,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
