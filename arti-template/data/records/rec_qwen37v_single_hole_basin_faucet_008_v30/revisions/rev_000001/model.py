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
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant, ~0.13 m tall, polished chrome.
# World frame: +Z up, deck at z = 0, spout/beak points toward +X (front).
# Tapered conical body, forward beak spout, hinged outlet aerator,
# subtle grip grooves on the body surface.
# ---------------------------------------------------------------------------

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.005

# Tapered conical body: wider at base, narrower at top.
BODY_R_BOTTOM = 0.028
BODY_R_TOP = 0.016
BODY_Z0 = 0.005  # just above flange
BODY_Z1 = 0.100  # top of body cone
BODY_H = BODY_Z1 - BODY_Z0

# Grip grooves: subtle circumferential rings on the body mid-section.
GROOVE_COUNT = 5
GROOVE_DEPTH = 0.0012
GROOVE_WIDTH = 0.003
GROOVE_ZONE_START = 0.30  # fraction of body height
GROOVE_ZONE_END = 0.65

# Beak spout: short forward projection from the upper body.
SPOUT_ORIGIN_Z = 0.082  # where spout exits the body
SPOUT_R = 0.012  # spout tube radius
SPOUT_LENGTH = 0.055  # forward reach
SPOUT_DROP = 0.025  # how far the tip curves downward

# Aerator disc at spout tip.
AERATOR_R = 0.013
AERATOR_THICK = 0.004

# Hinge limits for the aerator flip.
AERATOR_OPEN_ANGLE = math.radians(75.0)


def _body_radius_at(z: float) -> float:
    """Linear taper interpolation along the conical body."""
    t = max(0.0, min(1.0, (z - BODY_Z0) / BODY_H))
    return BODY_R_BOTTOM + t * (BODY_R_TOP - BODY_R_BOTTOM)


def _build_body_shape() -> cq.Workplane:
    """Tapered conical body with circumferential grip grooves cut in."""
    # Loft from bottom circle to top circle.
    body = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z0))
        .circle(BODY_R_BOTTOM)
        .workplane(offset=BODY_H)
        .circle(BODY_R_TOP)
        .loft()
    )

    # Cut grip grooves: thin circumferential channels around the mid body.
    for i in range(GROOVE_COUNT):
        frac = GROOVE_ZONE_START + (GROOVE_ZONE_END - GROOVE_ZONE_START) * i / max(
            1, GROOVE_COUNT - 1
        )
        z_groove = BODY_Z0 + frac * BODY_H
        r_local = _body_radius_at(z_groove)
        # A torus-like ring cutter slightly larger than the body surface.
        groove_cutter = (
            cq.Workplane("XY", origin=(0.0, 0.0, z_groove - GROOVE_WIDTH / 2.0))
            .circle(r_local + 0.001)
            .circle(r_local - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH)
        )
        body = body.cut(groove_cutter)

    return body


def _build_spout_shape() -> cq.Workplane:
    """Short forward beak spout: straight section then smooth downward curve,
    built in spout-local frame with origin at the body exit point; +X forward."""
    r_out = SPOUT_R
    shank_x0 = -0.008  # seated inside the body wall
    shank_x1 = 0.025
    bend = 0.020  # bend radius
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet rim at the downturned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.004))
        .circle(r_out * 0.95)
        .workplane(offset=-0.007)
        .circle(r_out * 1.25)
        .loft()
    )
    spout = tube.union(flare)

    # Hollow bore through the outlet.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.005))
        .circle(r_out * 1.05)
        .workplane(offset=0.014)
        .circle(r_out * 0.75)
        .loft()
    )
    return spout.cut(bore)


def _build_aerator_shape() -> cq.Workplane:
    """Small aerator disc with a thin hinge tab on one edge.
    Aerator-local frame: origin at disc center, z-axis up through disc face."""
    disc = cq.Workplane("XY").circle(AERATOR_R).extrude(AERATOR_THICK)
    # Small hinge tab protruding from one edge for the hinge pin connection.
    tab = (
        cq.Workplane("XY", origin=(AERATOR_R - 0.001, -0.003, 0.0))
        .box(0.006, 0.006, AERATOR_THICK, centered=(True, True, False))
    )
    aerator = disc.union(tab)
    # Small perforations to read as an aerator screen.
    for dx, dy in [
        (0.004, 0.004),
        (-0.004, 0.004),
        (0.004, -0.004),
        (-0.004, -0.004),
        (0.0, 0.0),
    ]:
        hole = (
            cq.Workplane("XY", origin=(dx, dy, -0.001))
            .circle(0.0015)
            .extrude(AERATOR_THICK + 0.002)
        )
        aerator = aerator.cut(hole)
    return aerator


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("aerator_screen", rgba=(0.30, 0.32, 0.36, 1.0))

    # ---------------- body (root): flange + tapered cone + groove rings ----
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_cone", tolerance=0.0003),
        material="chrome",
        name="body_cone",
    )

    # Decorative groove ring visuals (dark inset rings) for grip texture.
    for i in range(GROOVE_COUNT):
        frac = GROOVE_ZONE_START + (GROOVE_ZONE_END - GROOVE_ZONE_START) * i / max(
            1, GROOVE_COUNT - 1
        )
        z_g = BODY_Z0 + frac * BODY_H
        r_g = _body_radius_at(z_g) - GROOVE_DEPTH * 0.5
        body.visual(
            Cylinder(radius=r_g, length=GROOVE_WIDTH * 0.6),
            origin=Origin(xyz=(0.0, 0.0, z_g)),
            material="chrome_dark",
            name=f"grip_groove_{i}",
        )

    # Small cap/dome at the top of the conical body.
    body.visual(
        Cylinder(radius=BODY_R_TOP * 0.85, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z1 + 0.003)),
        material="chrome_brushed",
        name="body_top_cap",
    )

    # ---------------- spout (fixed): short forward beak -------------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout_beak", tolerance=0.0003),
        material="chrome",
        name="spout_beak",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_ORIGIN_Z)),
    )

    # ---------------- aerator (revolute hinge at spout tip) ----------------
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_shape(), "aerator_disc", tolerance=0.0003),
        material="aerator_screen",
        name="aerator_disc",
    )

    # The hinge sits at the rear edge of the spout outlet.
    # Spout tip world position (approximate):
    # spout local end_x ≈ 0.045, end_z ≈ -0.020 relative to spout origin.
    # Spout origin is at body (0, 0, SPOUT_ORIGIN_Z).
    spout_tip_x = 0.045
    spout_tip_z = SPOUT_ORIGIN_Z - 0.020
    # Aerator frame: disc hangs below the spout tip, hinge at rear edge.
    # Hinge origin is at the rear edge of the aerator disc.
    hinge_origin = Origin(xyz=(spout_tip_x - AERATOR_R + 0.003, 0.0, spout_tip_z - 0.003))

    # The aerator part frame sits at the hinge pin location.
    # In aerator local frame, the hinge pin is at x = -(AERATOR_R - 0.003), y=0, z=0.
    # The disc extends forward from there.

    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        # Hinge at spout outlet rear edge (in spout local frame).
        origin=Origin(xyz=(0.045 - AERATOR_R + 0.003, 0.0, -0.023)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=AERATOR_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    hinge = object_model.get_articulation("aerator_hinge")

    # ---- conical body: wider at base than at top --------------------------
    body_cone_aabb = ctx.part_element_world_aabb(body, elem="body_cone")
    ctx.check(
        "body is a tapered cone (exists and spans the expected height)",
        body_cone_aabb is not None
        and (body_cone_aabb[1][2] - body_cone_aabb[0][2]) > 0.07,
        details=f"body_cone aabb={body_cone_aabb}",
    )
    # Verify taper: the body should be wider in XY at the bottom than at the top.
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    top_cap_aabb = ctx.part_element_world_aabb(body, elem="body_top_cap")
    ctx.check(
        "body tapers: base region is wider than top cap",
        flange_aabb is not None
        and top_cap_aabb is not None
        and (flange_aabb[1][1] - flange_aabb[0][1])
        > (top_cap_aabb[1][1] - top_cap_aabb[0][1]) + 0.008,
        details=f"flange={flange_aabb}, top_cap={top_cap_aabb}",
    )

    # ---- base flange seated on deck ---------------------------------------
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    # ---- grip grooves exist on the body -----------------------------------
    groove_names = [f"grip_groove_{i}" for i in range(GROOVE_COUNT)]
    groove_aabbs = [ctx.part_element_world_aabb(body, elem=n) for n in groove_names]
    ctx.check(
        f"grip grooves present ({GROOVE_COUNT} circumferential rings)",
        all(a is not None for a in groove_aabbs),
        details=f"groove aabbs={groove_aabbs}",
    )
    # Grooves should span a vertical range on the body.
    if groove_aabbs[0] is not None and groove_aabbs[-1] is not None:
        groove_z_span = groove_aabbs[-1][1][2] - groove_aabbs[0][0][2]
        ctx.check(
            "grip grooves span a meaningful vertical range on the body",
            groove_z_span > 0.020,
            details=f"groove z span={groove_z_span:.4f}",
        )

    # ---- spout projects forward from the body -----------------------------
    spout_aabb = ctx.part_world_aabb(spout)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "spout beak reaches forward beyond the body",
        spout_aabb is not None
        and body_aabb is not None
        and spout_aabb[1][0] > body_aabb[1][0] + 0.015,
        details=f"spout aabb={spout_aabb}, body aabb={body_aabb}",
    )
    ctx.check(
        "spout tip droops downward from the exit height",
        spout_aabb is not None
        and spout_aabb[0][2] < SPOUT_ORIGIN_Z - 0.010,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- aerator hinge: non-fixed joint with correct limits ---------------
    hl = hinge.motion_limits
    ctx.check(
        "aerator hinge has positive open angle range",
        hl is not None
        and hl.lower is not None
        and hl.upper is not None
        and abs(hl.lower) < 1e-9
        and hl.upper > 0.5,
        details=f"hinge limits={hl}",
    )

    # ---- aerator exists near the spout tip --------------------------------
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator disc is positioned at the spout outlet",
        aerator_aabb is not None
        and spout_aabb is not None
        and aerator_aabb[1][0] > spout_aabb[0][0] + 0.020
        and aerator_aabb[0][2] < spout_aabb[1][2],
        details=f"aerator aabb={aerator_aabb}, spout aabb={spout_aabb}",
    )

    # ---- decisive pose: aerator flips open --------------------------------
    rest_aabb = ctx.part_world_aabb(aerator)
    with ctx.pose({hinge: AERATOR_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator flips open (disc bottom drops when hinge opens)",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < rest_aabb[0][2] - 0.008,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    # ---- overall height check ---------------------------------------------
    ctx.check(
        "overall faucet height is about 0.10 to 0.14 m",
        body_aabb is not None
        and 0.095 <= body_aabb[1][2] <= 0.145,
        details=f"body aabb={body_aabb}",
    )

    # ---- spout seated in body (intentional overlap) -----------------------
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_beak",
        elem_b="body_cone",
        reason="Spout shank is intentionally seated inside the solid body cone wall.",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank remains seated in body",
    )

    return ctx.report()


object_model = build_object_model()
