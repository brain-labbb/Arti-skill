from __future__ import annotations

"""M84-style stun (flashbang) grenade, about 0.13 m tall.

Smooth-shell variant: the body cylinder is a plain solid olive-green shell
with no vent holes, no visible charge tube, no brown mid-band, and no red
flash insert. Hex end caps, fuze block, safety lever, twin pull pins, and
pull rings are identical to the parent perforated model.

Articraft brief:
- Object: M84 stun grenade standing upright, ~0.131 m tall, body tube d=0.048,
  chunky hex end caps ~0.056 across corners.
- Root/support: `body` (root) = bottom hex cap + smooth olive shell + top hex
  cap with white stencil + silver fuze (collar, cylinder, lever lugs). Grounded
  at z=0.
- Articulations:
  1. lever_pivot   REVOLUTE  body -> safety_lever, pivot (0.010, 0, 0.128),
     axis (0,-1,0), 0..1.57 rad: positive q swings the folded lever outward/up.
  2. primary_pin_slide  PRISMATIC body -> primary_pin, axis +Y, 0..0.02 m.
  3. secondary_pin_slide PRISMATIC body -> secondary_pin, axis at 165 deg in
     the XY plane (frame yaw 75 deg, local +Y), 0..0.02 m.
  4/5. *_ring_swing REVOLUTE pin -> ring, about the pin shaft axis, +-1.05 rad
     (~120 deg span). Ring hoop is off the joint axis, proving rotation.
- Visible geometry: smooth olive-green cylindrical shell (no vent holes);
  '01-0009' stencil dashes on the top hex cap; weathered-silver fuze; two
  stainless wire pull rings d~0.035 splayed at different angles.
- Intentional overlaps: lever pivot pin in the body lugs, pin shafts seated
  through the fuze bore, rings threaded through pin eyes/shafts (allowances).
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
BODY_R_OUT = 0.024

HEX_R = 0.028  # circumradius; across-flats 0.0485, across-corners 0.056
CAP_BOT_Z0, CAP_BOT_Z1 = 0.0, 0.020
SHELL_Z0, SHELL_Z1 = 0.019, 0.091  # embeds 1 mm into each cap
CAP_TOP_Z0, CAP_TOP_Z1 = 0.090, 0.112

FUZE_COLLAR_R, FUZE_COLLAR_Z0, FUZE_COLLAR_Z1 = 0.013, 0.112, 0.1155
FUZE_BODY_R, FUZE_BODY_Z0, FUZE_BODY_Z1 = 0.0115, 0.1155, 0.1265

LEVER_PIVOT = (0.010, 0.0, 0.128)

PIN_SHAFT_R = 0.0016
PIN_TRAVEL = 0.02
PIN1_Z = 0.1215
PIN2_Z = 0.1175
PIN2_YAW = math.radians(75.0)  # local +Y points at 165 deg in world XY
PIN_MOUNT_R = 0.0135  # pin frame offset from the body axis

RING_MAJOR_R = 0.016  # ring outer diameter ~0.035 m
RING_WIRE_R = 0.0018

# ---------------------------------------------------------------- materials
OLIVE = Material(name="olive_steel", rgba=(0.33, 0.36, 0.20, 1.0))
OLIVE_DARK = Material(name="olive_cap", rgba=(0.29, 0.32, 0.18, 1.0))
SILVER = Material(name="fuze_silver", rgba=(0.62, 0.63, 0.65, 1.0))
STEEL = Material(name="stainless", rgba=(0.72, 0.73, 0.75, 1.0))
LEVER_GREEN = Material(name="lever_green", rgba=(0.15, 0.18, 0.11, 1.0))
WHITE = Material(name="stencil_white", rgba=(0.92, 0.92, 0.90, 1.0))


def _hex_prism(height: float) -> cq.Workplane:
    """Hex prism, flats facing +X/-X (vertices at 30 + 60k degrees)."""
    pts = [
        (HEX_R * math.cos(math.radians(30 + 60 * k)), HEX_R * math.sin(math.radians(30 + 60 * k)))
        for k in range(6)
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(height)


def _smooth_shell() -> cq.Workplane:
    """Solid olive cylinder, no vent holes (local z from 0)."""
    h = SHELL_Z1 - SHELL_Z0
    return cq.Workplane("XY").circle(BODY_R_OUT).extrude(h)


def _ring_mesh(name: str):
    """Round bent-wire pull ring in the local X-Z plane, passing the origin.

    Keeping the hoop in the pin-local X-Z plane keeps every wire point at the
    eye's radial offset from the grenade axis, so the ring clears the fuze.
    """
    pts = []
    n = 28
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((RING_MAJOR_R * math.sin(t), 0.0, 0.0155 - RING_MAJOR_R * math.cos(t)))
    geo = tube_from_spline_points(
        pts,
        radius=RING_WIRE_R,
        closed_spline=True,
        samples_per_segment=4,
        radial_segments=10,
    )
    return mesh_from_geometry(geo, name)


def _add_pin_visuals(pin) -> None:
    """Shaft along local -Y through the fuze, eye head at the outer end."""
    pin.visual(
        Cylinder(radius=PIN_SHAFT_R, length=0.029),
        origin=Origin(xyz=(0.0, -0.0145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="shaft",
    )
    pin.visual(
        Cylinder(radius=0.0033, length=0.003),
        origin=Origin(xyz=(0.0, 0.0005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="eye_head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="m84_stun_grenade")

    # ------------------------------------------------------------ body (root)
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_hex_prism(CAP_BOT_Z1 - CAP_BOT_Z0), "hex_cap_bottom"),
        origin=Origin(xyz=(0.0, 0.0, CAP_BOT_Z0)),
        material=OLIVE_DARK,
        name="hex_cap_bottom",
    )
    body.visual(
        mesh_from_cadquery(_smooth_shell(), "smooth_shell", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, SHELL_Z0)),
        material=OLIVE,
        name="smooth_shell",
    )
    body.visual(
        mesh_from_cadquery(_hex_prism(CAP_TOP_Z1 - CAP_TOP_Z0), "hex_cap_top"),
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z0)),
        material=OLIVE_DARK,
        name="hex_cap_top",
    )
    # '01-0009' white stencil dashes on the -X flat of the top hex cap.
    apothem = HEX_R * math.cos(math.radians(30.0))
    for i in range(7):
        y = 0.0085 - i * (0.017 / 6.0)
        body.visual(
            Box((0.0008, 0.0016, 0.005)),
            origin=Origin(xyz=(-(apothem + 0.0002), y, 0.103)),
            material=WHITE,
            name=f"stencil_char_{i}",
        )
    # Weathered silver fuze stack.
    body.visual(
        Cylinder(radius=FUZE_COLLAR_R, length=FUZE_COLLAR_Z1 - FUZE_COLLAR_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (FUZE_COLLAR_Z0 + FUZE_COLLAR_Z1))),
        material=SILVER,
        name="fuze_collar",
    )
    body.visual(
        Cylinder(radius=FUZE_BODY_R, length=FUZE_BODY_Z1 - FUZE_BODY_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (FUZE_BODY_Z0 + FUZE_BODY_Z1))),
        material=SILVER,
        name="fuze_body",
    )
    # Lever pivot lug ears either side of the lever plate.
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.006, 0.0025, 0.006)),
            origin=Origin(xyz=(LEVER_PIVOT[0], sy * 0.00775, LEVER_PIVOT[2])),
            material=SILVER,
            name=f"lever_lug_{i}",
        )

    # --------------------------------------------------------- safety lever
    # Local frame at the pivot; folded plate runs over the fuze top (+X) then
    # down the +X side of the body to mid-height.
    lever = model.part("safety_lever")
    lever.visual(
        Cylinder(radius=0.0016, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="pivot_pin",
    )
    lever.visual(
        Box((0.019, 0.012, 0.002)),
        origin=Origin(xyz=(0.0075, 0.0, 0.0)),
        material=LEVER_GREEN,
        name="top_plate",
    )
    lever.visual(
        Box((0.002, 0.012, 0.081)),
        origin=Origin(xyz=(0.017, 0.0, -0.0395)),
        material=LEVER_GREEN,
        name="side_strap",
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=LEVER_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=6.0, lower=0.0, upper=1.57),
    )

    # ------------------------------------------------------------ pull pins
    pin1 = model.part("primary_pin")
    _add_pin_visuals(pin1)
    model.articulation(
        "primary_pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin1,
        origin=Origin(xyz=(0.0, PIN_MOUNT_R, PIN1_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL),
    )

    pin2 = model.part("secondary_pin")
    _add_pin_visuals(pin2)
    d2 = (math.cos(math.radians(165.0)), math.sin(math.radians(165.0)))
    model.articulation(
        "secondary_pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin2,
        origin=Origin(
            xyz=(PIN_MOUNT_R * d2[0], PIN_MOUNT_R * d2[1], PIN2_Z),
            rpy=(0.0, 0.0, PIN2_YAW),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL),
    )

    # ------------------------------------------------------------ pull rings
    ring1 = model.part("primary_pull_ring")
    ring1.visual(
        _ring_mesh("primary_pull_ring"),
        origin=Origin(rpy=(0.0, 0.6, 0.0)),  # draped toward the lever side
        material=STEEL,
        name="ring_loop",
    )
    model.articulation(
        "primary_ring_swing",
        ArticulationType.REVOLUTE,
        parent=pin1,
        child=ring1,
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=-1.05, upper=1.05),
    )

    ring2 = model.part("secondary_pull_ring")
    ring2.visual(
        _ring_mesh("secondary_pull_ring"),
        origin=Origin(rpy=(0.0, 0.4, 0.0)),  # splayed on its own 75-deg axis
        material=STEEL,
        name="ring_loop",
    )
    model.articulation(
        "secondary_ring_swing",
        ArticulationType.REVOLUTE,
        parent=pin2,
        child=ring2,
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lever = object_model.get_part("safety_lever")
    pin1 = object_model.get_part("primary_pin")
    pin2 = object_model.get_part("secondary_pin")
    ring1 = object_model.get_part("primary_pull_ring")
    ring2 = object_model.get_part("secondary_pull_ring")

    j_lever = object_model.get_articulation("lever_pivot")
    j_pin1 = object_model.get_articulation("primary_pin_slide")
    j_pin2 = object_model.get_articulation("secondary_pin_slide")
    j_ring1 = object_model.get_articulation("primary_ring_swing")
    j_ring2 = object_model.get_articulation("secondary_ring_swing")

    # ------------------------------------------------- intentional overlaps
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_lug_0",
        elem_b="pivot_pin",
        reason="Hinge pin intentionally captured inside the fuze lug ear.",
    )
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_lug_1",
        elem_b="pivot_pin",
        reason="Hinge pin intentionally captured inside the fuze lug ear.",
    )
    for pin in (pin1, pin2):
        ctx.allow_overlap(
            body,
            pin,
            elem_a="fuze_body",
            elem_b="shaft",
            reason="Safety pin shaft seated through the fuze block bore proxy.",
        )
    for pin, ring in ((pin1, ring1), (pin2, ring2)):
        ctx.allow_overlap(
            pin,
            ring,
            elem_a="eye_head",
            elem_b="ring_loop",
            reason="Pull ring wire threaded through the pin eye.",
        )
        ctx.allow_overlap(
            pin,
            ring,
            elem_a="shaft",
            elem_b="ring_loop",
            reason="Pull ring wire passes the shaft end at the pin eye.",
        )

    # ------------------------------------------------------- size/grounding
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body grounded at z=0",
        aabb is not None and abs(aabb[0][2]) < 0.002,
        details=f"aabb={aabb}",
    )
    ctx.check(
        "overall height about 0.13 m",
        aabb is not None and 0.120 <= aabb[1][2] <= 0.140,
        details=f"aabb={aabb}",
    )
    shell_aabb = ctx.part_element_world_aabb(body, elem="smooth_shell")
    ctx.check(
        "body tube about 0.05 m across",
        shell_aabb is not None
        and 0.044 <= (shell_aabb[1][0] - shell_aabb[0][0]) <= 0.052
        and 0.044 <= (shell_aabb[1][1] - shell_aabb[0][1]) <= 0.052,
        details=f"shell aabb={shell_aabb}",
    )

    # ----------------------------------------------------- key part layout
    stencil = ctx.part_element_world_aabb(body, elem="stencil_char_0")
    ctx.check(
        "stencil sits proud on the -X hex flat of the top cap",
        stencil is not None and stencil[0][0] < -0.024 and 0.090 < stencil[0][2] < 0.112,
        details=f"stencil aabb={stencil}",
    )

    # Smooth-shell variant: no vent holes, no charge tube, no band, no insert.
    body_visual_names = [v.name for v in body.visuals]
    ctx.check(
        "smooth shell replaces perforated shell",
        "smooth_shell" in body_visual_names and "perforated_shell" not in body_visual_names,
        details=f"body visuals={body_visual_names}",
    )
    ctx.check(
        "no charge tube, mid band, or flash insert on smooth variant",
        all(
            n not in body_visual_names
            for n in ("charge_tube", "mid_band", "flash_insert")
        ),
        details=f"body visuals={body_visual_names}",
    )

    # Lever hinge pin is captured by the lug ears; pins seat in the fuze.
    ctx.expect_contact(lever, body, elem_a="pivot_pin", elem_b="lever_lug_0", contact_tol=1e-6)
    ctx.expect_contact(pin1, body, elem_a="shaft", elem_b="fuze_body", contact_tol=1e-6)
    ctx.expect_contact(pin2, body, elem_a="shaft", elem_b="fuze_body", contact_tol=1e-6)
    # Each ring is threaded onto its pin eye.
    ctx.expect_contact(ring1, pin1, elem_a="ring_loop", elem_b="eye_head", contact_tol=1e-6)
    ctx.expect_contact(ring2, pin2, elem_a="ring_loop", elem_b="eye_head", contact_tol=1e-6)

    # ------------------------------------------------------ joint contracts
    ctx.check(
        "lever joint is revolute about -Y, ~90 deg",
        j_lever.articulation_type == ArticulationType.REVOLUTE
        and j_lever.axis[1] < -0.9
        and j_lever.motion_limits is not None
        and abs(j_lever.motion_limits.upper - 1.57) < 0.05
        and abs(j_lever.motion_limits.lower) < 1e-9,
    )
    for j, label in ((j_pin1, "primary"), (j_pin2, "secondary")):
        ctx.check(
            f"{label} pin is prismatic with 0.02 m travel",
            j.articulation_type == ArticulationType.PRISMATIC
            and j.motion_limits is not None
            and abs(j.motion_limits.upper - PIN_TRAVEL) < 1e-6
            and abs(j.motion_limits.lower) < 1e-9,
        )
    ctx.check(
        "pin axes are distinct horizontal directions (~75 deg apart)",
        abs(j_pin1.origin.rpy[2] - j_pin2.origin.rpy[2]) > math.radians(50.0),
        details=f"yaw1={j_pin1.origin.rpy[2]}, yaw2={j_pin2.origin.rpy[2]}",
    )
    for j, parent_name in ((j_ring1, "primary_pin"), (j_ring2, "secondary_pin")):
        ctx.check(
            f"ring joint on {parent_name} is revolute, ~120 deg span",
            j.articulation_type == ArticulationType.REVOLUTE
            and j.parent == parent_name
            and j.motion_limits is not None
            and abs((j.motion_limits.upper - j.motion_limits.lower) - 2.1) < 0.05,
        )

    # ---------------------------------------------------------- pose checks
    lever_rest = ctx.part_world_aabb(lever)
    with ctx.pose({j_lever: 1.4}):
        lever_open = ctx.part_world_aabb(lever)
    ctx.check(
        "lever swings outward and up off the body side",
        lever_rest is not None
        and lever_open is not None
        and lever_open[0][2] > lever_rest[0][2] + 0.05
        and lever_open[1][0] > lever_rest[1][0] + 0.03,
        details=f"rest={lever_rest}, open={lever_open}",
    )

    r1_rest = ctx.part_world_position(ring1)
    with ctx.pose({j_pin1: PIN_TRAVEL}):
        r1_out = ctx.part_world_position(ring1)
    ctx.check(
        "primary pin extracts +Y carrying its ring",
        r1_rest is not None and r1_out is not None and r1_out[1] - r1_rest[1] > 0.018,
        details=f"rest={r1_rest}, out={r1_out}",
    )

    r2_rest = ctx.part_world_position(ring2)
    with ctx.pose({j_pin2: PIN_TRAVEL}):
        r2_out = ctx.part_world_position(ring2)
    ctx.check(
        "secondary pin extracts on its own 165-deg axis",
        r2_rest is not None and r2_out is not None and r2_rest[0] - r2_out[0] > 0.017,
        details=f"rest={r2_rest}, out={r2_out}",
    )

    # Off-axis hoop proves the ring really rotates about the pin shaft.
    ring1_rest = ctx.part_world_aabb(ring1)
    with ctx.pose({j_ring1: 1.0}):
        ring1_swung = ctx.part_world_aabb(ring1)
    ctx.check(
        "ring hoop (off-axis) swings about the pin shaft axis",
        ring1_rest is not None
        and ring1_swung is not None
        and abs(
            0.5 * (ring1_swung[0][0] + ring1_swung[1][0])
            - 0.5 * (ring1_rest[0][0] + ring1_rest[1][0])
        )
        > 0.004,
        details=f"rest={ring1_rest}, swung={ring1_swung}",
    )

    return ctx.report()


object_model = build_object_model()
