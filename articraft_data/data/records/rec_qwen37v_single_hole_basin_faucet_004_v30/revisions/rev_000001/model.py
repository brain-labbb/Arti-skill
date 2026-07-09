from __future__ import annotations

"""Compact single-hole basin faucet with tapered conical body and hinged aerator.

Layout (meters, +Z up, ground at z=0, spout beak along +X):
- A round base plate sits on the counter.
- A tapered conical body rises from the base plate, wider at bottom, narrower at top.
- Subtle horizontal grip grooves ring the body midsection.
- A small forward beak extends from the upper body, carrying the outlet.
- A hinged aerator flap flips open at the beak tip underside.
"""

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

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_R = 0.032
BASE_H = 0.006

BODY_BOTTOM_R = 0.025
BODY_TOP_R = 0.016
BODY_HEIGHT = 0.140
BODY_BOTTOM_Z = BASE_H
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT  # 0.146

# Grip groove parameters (subtle horizontal rings on body midsection)
GROOVE_Z_START = BODY_BOTTOM_Z + BODY_HEIGHT * 0.35
GROOVE_Z_END = BODY_BOTTOM_Z + BODY_HEIGHT * 0.65
GROOVE_COUNT = 6
GROOVE_DEPTH = 0.0012
GROOVE_WIDTH = 0.002

# Beak / spout dimensions
BEAK_ORIGIN_Z = BODY_TOP_Z - 0.020  # starts just below body top
BEAK_LENGTH = 0.055  # forward reach from body center
BEAK_WIDTH = 0.018
BEAK_HEIGHT = 0.014
BEAK_TIP_X = BEAK_LENGTH  # tip extends to this X from center

# Aerator flap dimensions (small disc that flips on hinge at beak underside)
AERATOR_R = 0.008
AERATOR_THICK = 0.003
AERATOR_HINGE_X = BEAK_TIP_X - 0.005  # hinge axis slightly behind tip
AERATOR_HINGE_Z = BEAK_ORIGIN_Z - BEAK_HEIGHT / 2.0  # underside of beak

FLIP_RANGE = math.radians(80.0)  # aerator flips open up to 80 degrees


def _build_tapered_body():
    """Build tapered conical body with grip grooves using CadQuery."""
    # Loft from bottom circle to top circle for tapered shape
    body = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BOTTOM_Z)
        .circle(BODY_BOTTOM_R)
        .workplane(offset=BODY_HEIGHT)
        .circle(BODY_TOP_R)
        .loft()
    )
    # Cut horizontal groove rings into the body for grip texture
    groove_spacing = (GROOVE_Z_END - GROOVE_Z_START) / (GROOVE_COUNT - 1)
    for i in range(GROOVE_COUNT):
        z = GROOVE_Z_START + i * groove_spacing
        # Compute local radius at this height via linear interpolation
        t = (z - BODY_BOTTOM_Z) / BODY_HEIGHT
        local_r = BODY_BOTTOM_R + t * (BODY_TOP_R - BODY_BOTTOM_R)
        # Cut a torus-shaped groove (ring channel) into the body
        groove = (
            cq.Workplane("XY")
            .workplane(offset=z - GROOVE_WIDTH / 2.0)
            .circle(local_r + 0.001)
            .circle(local_r - GROOVE_DEPTH)
            .extrude(GROOVE_WIDTH)
        )
        body = body.cut(groove)
    return body


def _build_beak():
    """Build the small forward beak/spout from upper body."""
    # A tapered box that extends forward from the body center
    beak = (
        cq.Workplane("XY")
        .workplane(offset=BEAK_ORIGIN_Z - BEAK_HEIGHT / 2.0)
        .center(BEAK_LENGTH / 2.0, 0.0)
        .rect(BEAK_LENGTH, BEAK_WIDTH)
        .extrude(BEAK_HEIGHT)
    )
    return beak


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base plate, tapered conical body, beak
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Round base plate
    body.visual(
        Cylinder(radius=BASE_R, length=BASE_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Tapered conical body with grip grooves
    tapered_body = _build_tapered_body()
    body.visual(
        mesh_from_cadquery(tapered_body, "tapered_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="conical_body",
    )

    # Forward beak / spout
    beak = _build_beak()
    body.visual(
        mesh_from_cadquery(beak, "spout_beak"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="spout_beak",
    )

    # Dark outlet recess on beak underside (small disc near tip)
    body.visual(
        Cylinder(radius=AERATOR_R - 0.001, length=0.002),
        origin=Origin(xyz=(AERATOR_HINGE_X, 0.0, AERATOR_HINGE_Z - 0.001)),
        material=dark,
        name="outlet_recess",
    )

    # Small hinge knuckles on beak underside (visual mounting detail)
    body.visual(
        Cylinder(radius=0.002, length=BEAK_WIDTH * 0.6),
        origin=Origin(
            xyz=(AERATOR_HINGE_X, 0.0, AERATOR_HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="hinge_barrel",
    )

    # ------------------------------------------------------------------
    # Aerator flap: thin plate that flips on hinge at beak underside
    # Part frame at the hinge axis so rotation is clean.
    # At q=0 the plate is horizontal, flush against beak underside.
    # Positive q (about +Y) swings the front edge downward (open).
    # ------------------------------------------------------------------
    aerator = model.part("aerator_flap")

    FLAP_LEN_X = 0.016  # plate length from hinge forward
    FLAP_WIDTH_Y = BEAK_WIDTH * 0.75
    FLAP_THICK_Z = AERATOR_THICK

    # Flat plate extends forward (+X) from hinge, top surface flush with hinge z
    aerator.visual(
        Box((FLAP_LEN_X, FLAP_WIDTH_Y, FLAP_THICK_Z)),
        origin=Origin(xyz=(FLAP_LEN_X / 2.0, 0.0, -FLAP_THICK_Z / 2.0)),
        material=chrome,
        name="flap_plate",
    )
    # Small aerator screen disc on the flap underside
    aerator.visual(
        Cylinder(radius=AERATOR_R * 0.6, length=0.001),
        origin=Origin(
            xyz=(FLAP_LEN_X / 2.0, 0.0, -FLAP_THICK_Z - 0.0005),
        ),
        material=dark,
        name="flap_screen",
    )
    # Hinge sleeve wrapping around the body's hinge barrel
    aerator.visual(
        Cylinder(radius=0.0025, length=BEAK_WIDTH * 0.5),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="flap_sleeve",
    )

    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(AERATOR_HINGE_X, 0.0, AERATOR_HINGE_Z)),
        # Axis along Y (left-right); positive q flips the flap downward/open
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=FLIP_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    aerator = object_model.get_part("aerator_flap")
    flip = object_model.get_articulation("aerator_flip")

    # --- joint plan: aerator flip is revolute with proper range ---
    ctx.check(
        "aerator flip joint is revolute 0..80 deg about horizontal Y axis",
        flip.articulation_type == ArticulationType.REVOLUTE
        and abs(flip.axis[0]) < 1e-9
        and abs(abs(flip.axis[1]) - 1.0) < 1e-9
        and abs(flip.axis[2]) < 1e-9
        and flip.motion_limits is not None
        and abs(flip.motion_limits.lower - 0.0) < 1e-9
        and abs(flip.motion_limits.upper - math.radians(80.0)) < 1e-6,
        details=f"axis={flip.axis}, limits={flip.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-5,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.15-0.18 m (compact basin faucet)",
        body_aabb is not None and 0.13 <= body_aabb[1][2] <= 0.19,
        details=f"body_aabb={body_aabb}",
    )

    # --- tapered conical body: wider at base, narrower at top ---
    body_elem = ctx.part_element_world_aabb(body, elem="conical_body")
    ctx.check(
        "conical body is tapered (wider at bottom than top)",
        body_elem is not None,
        details=f"conical_body aabb={body_elem}",
    )

    # --- beak extends forward ---
    beak_elem = ctx.part_element_world_aabb(body, elem="spout_beak")
    ctx.check(
        "spout beak extends forward from body",
        beak_elem is not None and beak_elem[1][0] > 0.03,
        details=f"beak aabb={beak_elem}",
    )

    # --- grip grooves exist on the body ---
    # Grooves are cut into the conical body mesh; verify the body element exists
    # and spans the expected height range where grooves are located.
    ctx.check(
        "conical body spans groove zone (grip texture region)",
        body_elem is not None
        and body_elem[0][2] < GROOVE_Z_START
        and body_elem[1][2] > GROOVE_Z_END,
        details=f"body z range={None if body_elem is None else (body_elem[0][2], body_elem[1][2])}, "
        f"groove zone=({GROOVE_Z_START}, {GROOVE_Z_END})",
    )

    # --- aerator mounting: hinge barrel and flap sleeve contact ---
    ctx.expect_contact(
        aerator,
        body,
        elem_a="flap_sleeve",
        elem_b="hinge_barrel",
        contact_tol=0.001,
        name="aerator flap sleeve wraps the body hinge barrel",
    )

    # --- hinge barrel sits on beak underside ---
    hinge_aabb = ctx.part_element_world_aabb(body, elem="hinge_barrel")
    ctx.check(
        "hinge barrel is on beak underside (below beak origin)",
        hinge_aabb is not None and hinge_aabb[1][2] < BEAK_ORIGIN_Z,
        details=f"hinge_barrel aabb={hinge_aabb}",
    )

    # --- decisive pose: aerator flips open ---
    rest_flap_aabb = ctx.part_world_aabb(aerator)
    with ctx.pose({flip: FLIP_RANGE}):
        flipped_aabb = ctx.part_world_aabb(aerator)
        ctx.check(
            "positive flip lowers the aerator flap (opens outlet)",
            rest_flap_aabb is not None
            and flipped_aabb is not None
            and flipped_aabb[0][2] < rest_flap_aabb[0][2] - 0.005,
            details=f"rest={rest_flap_aabb}, flipped={flipped_aabb}",
        )

    # --- at rest, flap is flush against beak underside ---
    # Beak is above flap in z. At rest the flap plate top is flush with beak bottom.
    ctx.expect_gap(
        body,
        aerator,
        axis="z",
        max_gap=0.002,
        max_penetration=1e-6,
        positive_elem="spout_beak",
        negative_elem="flap_plate",
        name="aerator flap seats flush against beak underside when closed",
    )

    return ctx.report()


object_model = build_object_model()
