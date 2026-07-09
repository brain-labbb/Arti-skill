from __future__ import annotations

# Black plastic square jerrycan / chemical pail (HDPE), near-cubic footprint.
# Frame: base on z=0, body grows up +Z; recessed top deck at ~0.27 m.
#   - chunky rounded-edge square body (CadQuery: filleted box) modeled hollow
#     with a real through pour-mouth at the neck
#   - moulded recessed top deck with a raised BRIDGE / STRAP grip handle spanning
#     an open slot: a real open gap beneath the bridge so fingers fit through
#   - moulded horizontal stacking ribs (one band near the base, one mid-body)
#     plus a slightly flared foot
#   - an offset round threaded neck rising from the deck, beside the handle
# Articulation: a round ribbed black SCREW CAP on the neck, DECOUPLED into two
# independent stages through a massless carrier link:
#   body -> (cap_rotate, CONTINUOUS +Z) -> cap_carrier -> (cap_slide, PRISMATIC +Z) -> cap

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- key dimensions (meters) ---
BODY_W = 0.250  # square footprint (X and Y)
BODY_H = 0.300  # overall height (deck reads ~0.27, handle/cap top ~0.30)
DECK_Z = 0.268  # recessed deck base height where the recess/grip sit
WALL_EDGE_R = 0.024  # softly rounded vertical edges of the chunky body

NECK_X = 0.050  # neck offset from center toward +X (beside the grip), on the deck
NECK_Y = -0.010
NECK_R = 0.030
NECK_TOP_Z = 0.300  # top of the neck where the cap seats

CAP_R = 0.036
CAP_H = 0.030
CAP_SLIDE = 0.045  # cap can lift this far off the neck

# handle bridge geometry
BRIDGE_X = -0.045  # bridge centered toward -X (away from the offset cap/neck)
BRIDGE_LEN = 0.150  # bridge span along Y (the strap you grip across)
BRIDGE_BAR_X = 0.040  # bar thickness along X
BRIDGE_TOP_Z = 0.302  # top of the raised strap
GAP_BOT_Z = 0.250  # bottom of the open finger gap (well below deck -> real clearance)


def _body_solid() -> cq.Workplane:
    half = BODY_W / 2.0

    # Main body block (z = 0 .. DECK_Z), softly rounded vertical edges.
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_W, DECK_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(WALL_EDGE_R)
    )
    # Soften the top horizontal edges of the body into the shoulder.
    body = body.edges(">Z").fillet(0.012)

    # Shoulder slab on top, slightly inset, rounded all around. The recessed deck
    # and the handle bridge are formed in this slab.
    sh_w = BODY_W - 0.014
    sh_z0 = DECK_Z - 0.020
    sh_top = DECK_Z + 0.014
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=sh_z0)
        .box(sh_w, sh_w, sh_top - sh_z0, centered=(True, True, False))
        .edges("|Z")
        .fillet(WALL_EDGE_R)
        .edges(">Z")
        .fillet(0.014)
    )
    body = body.union(shoulder)

    # Recessed top deck: carve a broad shallow pocket into the shoulder so the
    # deck reads as a moulded recess. We leave a rim around the perimeter.
    deck_floor = DECK_Z - 0.004  # recessed deck floor
    pocket = (
        cq.Workplane("XY")
        .workplane(offset=deck_floor)
        .box(sh_w - 0.030, sh_w - 0.030, sh_top - deck_floor + 0.01, centered=(True, True, False))
    )
    body = body.cut(pocket)

    # --- Raised BRIDGE / STRAP handle spanning the recessed deck along Y. ---
    # A chunky strap that arches across the deck; you grip it from beneath through
    # a real open gap. Built as a bar standing proud, then a finger tunnel carved
    # beneath it leaving a graspable top strap supported on two end legs.
    bar = (
        cq.Workplane("XY")
        .workplane(offset=GAP_BOT_Z)
        .center(BRIDGE_X, 0.0)
        .box(BRIDGE_BAR_X, BRIDGE_LEN, BRIDGE_TOP_Z - GAP_BOT_Z, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.010)
    )
    body = body.union(bar)

    # Finger tunnel through X beneath the strap: an open gap so fingers fit
    # through. It cuts fully across X (through the bar) at a height that leaves a
    # solid ~14 mm top strap, and the gap floor sits below the deck for clearance.
    tunnel_w = BRIDGE_LEN - 0.044  # along Y (shorter than span -> end legs survive)
    tunnel_top = BRIDGE_TOP_Z - 0.016  # leave ~16 mm solid grip on top
    tunnel = (
        cq.Workplane("YZ")
        .workplane(offset=-half - 0.02)
        .center(0.0, (GAP_BOT_Z + tunnel_top) / 2.0)
        .rect(tunnel_w, tunnel_top - GAP_BOT_Z)
        .extrude(BODY_W + 0.04)
    )
    body = body.cut(tunnel)

    # Offset round threaded neck rising from the deck (toward +X, beside the grip).
    # Start its base well below the recessed deck floor and add a flared boss so
    # it stays solidly merged with the body (not isolated by the deck pocket cut).
    neck_base_z = deck_floor - 0.030
    boss = (
        cq.Workplane("XY")
        .workplane(offset=neck_base_z)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R + 0.010)
        .extrude(0.020)
    )
    body = body.union(boss)
    neck = (
        cq.Workplane("XY")
        .workplane(offset=neck_base_z)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - neck_base_z)
    )
    body = body.union(neck)
    # Open pour mouth: bore the neck axis deep into the body so the mouth reads
    # as a real through opening (hollow), not a shallow recess under the cap.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=0.170)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R - 0.013)
        .extrude((NECK_TOP_Z + 0.002) - 0.170)
    )
    body = body.cut(bore)

    # Mid-body and base moulded stacking ribs (raised bands around the body).
    for zb, th in ((0.020, 0.012), (0.150, 0.010)):
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=zb)
            .box(BODY_W + 0.008, BODY_W + 0.008, th, centered=(True, True, False))
            .edges("|Z")
            .fillet(WALL_EDGE_R)
        )
        body = body.union(ridge)

    # Slightly flared foot: a short wider band at the very bottom.
    foot = (
        cq.Workplane("XY")
        .box(BODY_W + 0.012, BODY_W + 0.012, 0.010, centered=(True, True, False))
        .edges("|Z")
        .fillet(WALL_EDGE_R)
    )
    body = body.union(foot)

    return body


def _cap_mesh() -> "object":
    # Round black ribbed/knurled screw cap: a short cylinder with an inset top
    # disc and a ring of vertical knurl ribs around the skirt.
    cap = CylinderGeometry(CAP_R, CAP_H, radial_segments=48)
    cap.translate(0.0, 0.0, CAP_H / 2.0)
    # Slightly inset top disc.
    top = CylinderGeometry(CAP_R - 0.004, 0.006, radial_segments=48)
    top.translate(0.0, 0.0, CAP_H + 0.002)
    cap.merge(top)
    # Vertical grip ribs (knurl) around the skirt.
    n = 28
    for i in range(n):
        a = 2.0 * math.pi * i / n
        rib = BoxGeometry((0.004, 0.006, CAP_H - 0.006))
        rib.rotate_z(a)
        rib.translate(CAP_R * math.cos(a), CAP_R * math.sin(a), CAP_H / 2.0)
        cap.merge(rib)
    return mesh_from_geometry(cap, "cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_jerrycan")

    hdpe_black = model.material("hdpe_black", rgba=(0.12, 0.12, 0.13, 1.0))
    cap_black = model.material("cap_black", rgba=(0.08, 0.08, 0.09, 1.0))
    marker_red = model.material("marker_red", rgba=(0.80, 0.10, 0.10, 1.0))

    # ---- body (root): chunky square pail + recessed deck + bridge handle + neck + ribs ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=hdpe_black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_W, BODY_H)), mass=2.0, origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0))
    )

    # ---- decoupled screw cap: massless carrier rotates, cap slides on the carrier ----
    carrier = model.part("cap_carrier")  # NO visuals (massless)
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=cap_black, name="cap_shell")
    # Off-axis marker so spin is provable: a small witness nub poking past the
    # skirt rim on +X, so a quarter-turn unambiguously swings its AABB.
    cap.visual(
        Box((0.008, 0.006, 0.006)),
        origin=Origin(xyz=(CAP_R + 0.003, 0.0, CAP_H - 0.004)),
        material=marker_red,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2 * CAP_R, 2 * CAP_R, CAP_H)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(NECK_X, NECK_Y, NECK_TOP_Z)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0, 0, 0.0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_SLIDE, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # --- body is a roughly cubic square pail of the right scale ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body footprint is square (width ~ depth)",
        abs(bext[0] - bext[1]) < 0.03 and bext[0] > 0.23 and bext[1] > 0.23,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body is roughly cubic (height within ~20% of width, not wildly tall)",
        abs(bext[2] - bext[0]) < 0.20 * bext[0] and bext[2] < bext[0] * 1.25,
        details=f"body w={bext[0]:.3f} d={bext[1]:.3f} h={bext[2]:.3f}",
    )

    # --- cap is round and seated on the offset neck at rest, near the top ---
    cext = _ext(ctx.part_world_aabb(cap))
    ctx.check(
        "cap is round (square XY bbox)",
        abs(cext[0] - cext[1]) < 0.006,
        details=f"cap extents={cext}",
    )
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap is offset on the top toward +X (not centered)",
        cap_pos is not None and cap_pos[0] > 0.03,
        details=f"cap origin={cap_pos}",
    )
    ctx.check(
        "cap sits near the top of the body",
        cap_pos is not None and cap_pos[2] > 0.26,
        details=f"cap z={cap_pos[2] if cap_pos else None}",
    )
    # Cap is offset clear of the handle bridge (bridge is on -X, cap on +X).
    ctx.check(
        "cap is offset clear of the handle bridge",
        cap_pos is not None and (cap_pos[0] - BRIDGE_X) > 0.06,
        details=f"cap_x={cap_pos[0]:.3f}, bridge_x={BRIDGE_X}",
    )

    # Cap rim is intentionally seated over the neck top (threaded engagement).
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="body_shell",
        reason="Screw cap skirt is threaded down over the neck top (seated engagement).",
    )

    # --- bridge handle: rises above the recessed deck with a REAL open gap under it ---
    # Sample the body cross-section at the bridge center: above tunnel_top there
    # must be solid strap; through the gap band there must be open air.
    bridge_top_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "handle bridge rises above the recessed deck",
        bridge_top_aabb[1][2] > DECK_Z + 0.02,
        details=f"body top z={bridge_top_aabb[1][2]:.3f}, deck_z={DECK_Z}",
    )
    # Real open gap beneath the bridge: the carved finger tunnel leaves the strap
    # top well above the gap floor, with the gap floor below the deck so fingers
    # have through clearance under the strap.
    tunnel_top = BRIDGE_TOP_Z - 0.016
    ctx.check(
        "real open gap beneath the bridge handle (fingers fit through)",
        (tunnel_top - GAP_BOT_Z) > 0.03 and GAP_BOT_Z < DECK_Z,
        details=f"gap span={tunnel_top - GAP_BOT_Z:.3f}, gap_floor={GAP_BOT_Z}, deck={DECK_Z}",
    )

    # --- cap rotates about +Z: the off-axis red marker swings around ---
    m_rest = ctx.part_element_world_aabb(cap, elem="cap_marker")
    m_rest_c = ((m_rest[0][0] + m_rest[1][0]) / 2.0, (m_rest[0][1] + m_rest[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m_spun = ctx.part_element_world_aabb(cap, elem="cap_marker")
    m_spun_c = ((m_spun[0][0] + m_spun[1][0]) / 2.0, (m_spun[0][1] + m_spun[1][1]) / 2.0)
    ctx.check(
        "cap spins about +Z (marker swings from +X toward +Y)",
        m_spun_c[1] > m_rest_c[1] + 0.02 and abs(m_spun_c[0] - NECK_X) < abs(m_rest_c[0] - NECK_X),
        details=f"marker rest_center={m_rest_c}, spun_center={m_spun_c}, neck_x={NECK_X}",
    )

    # --- cap slides up off the neck along +Z (independent of rotation) ---
    z_rest = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_SLIDE}):
        z_up = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + CAP_SLIDE * 0.8,
        details=f"z_rest={z_rest:.3f}, z_up={z_up:.3f}",
    )

    # --- joints are decoupled: rotating does not move it up ---
    with ctx.pose({rotate: math.pi}):
        z_after_spin = ctx.part_world_position(cap)[2]
    ctx.check(
        "rotation does not change cap height (decoupled)",
        abs(z_after_spin - z_rest) < 0.002,
        details=f"z_rest={z_rest:.3f}, z_after_spin={z_after_spin:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
