from __future__ import annotations

# Twist-action retractable marker/highlighter.
#
# Variant of the STABILO BOSS highlighter: the separate cap is removed and
# replaced with a twist-action retraction mechanism. The barrel is split into
# a fixed front body and a rotating rear twist collar. Twisting the collar
# (REVOLUTE around +X) drives the chisel nib carrier forward (PRISMATIC along
# +X, mimic of the twist joint) from retracted (nib hidden inside the rounded
# nose) to extended (nib protruding past the front opening).
#
# Frame convention:
#   +X = pen length (front at +X, rear at -X)
#   Cross-section in Y-Z plane (Y = width, Z = height)
#   front_body part frame origin at the rear seam plane (X=0)

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BODY_LEN = 0.062          # front body barrel length
BODY_W = 0.0170           # barrel width (Y)
BODY_H = 0.0120           # barrel height (Z)
BODY_CORNER_R = 0.0030

NOSE_LEN = 0.014          # tapered nose from barrel to opening
NOSE_OPENING_W = 0.0100   # opening width at nose tip
NOSE_OPENING_H = 0.0070   # opening height at nose tip

COLLAR_LEN = 0.035        # rear twist collar length
COLLAR_W = BODY_W
COLLAR_H = BODY_H
COLLAR_CORNER_R = BODY_CORNER_R

KNURL_LEN = 0.010         # knurled grip band length at the seam
KNURL_RAISE = 0.0006      # how far the band stands proud of the collar
N_RIBS = 16               # longitudinal grip ribs on the knurl band

NIB_BASE_LEN = 0.0080     # straight black holder section
NIB_BASE_W = 0.0100
NIB_BASE_H = 0.0075
NIB_WEDGE_LEN = 0.0100    # tapered chisel section
NIB_TIP_W = 0.0080
NIB_TIP_H = 0.0018        # thin chisel edge

CARRIER_STEM_LEN = 0.026  # hidden stem inside barrel for support
CARRIER_STEM_W = 0.006
CARRIER_STEM_H = 0.005

NIB_TRAVEL = 0.014        # total nib extension travel (retracted → extended)

CLIP_LEN = 0.032          # pocket clip length
CLIP_W = 0.0070
CLIP_THICK = 0.0016

# Derived positions (from front_body frame origin at rear seam X=0)
NOSE_TIP_X = BODY_LEN + NOSE_LEN                          # 0.076
NIB_TOTAL_LEN = NIB_BASE_LEN + NIB_WEDGE_LEN              # 0.018
# Nib retracted: tip just inside the nose opening
NIB_RETRACT_X = NOSE_TIP_X - NIB_TOTAL_LEN - 0.002       # 0.056
# Bore for the nib channel through the barrel + nose
BORE_START_X = 0.018
BORE_W = NIB_BASE_W + 0.004                               # 0.014
BORE_H = NIB_BASE_H + 0.004                               # 0.0115
BORE_LEN = BODY_LEN - BORE_START_X + NOSE_LEN + 0.002     # through nose

# Twist collar rotation for full nib extension
TWIST_UPPER = 2.0 * math.pi  # one full turn

# Materials
LIME = (0.82, 0.93, 0.13, 1.0)         # fluorescent yellow-green body
DARK_LIME = (0.65, 0.78, 0.08, 1.0)    # twist collar (slightly darker)
BLACK = (0.07, 0.07, 0.08, 1.0)        # nib + clip
DARK_GRAY = (0.22, 0.22, 0.24, 1.0)    # knurl band + ribs
FELT_COLOR = (0.78, 0.90, 0.15, 1.0)   # ink-soaked felt tip


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rounded_rect_prism(length, width, height, corner_r):
    """Rounded-rect prism, long axis +X, centered Y/Z, X in [0, length]."""
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .extrude(length)
        .edges("|X")
        .fillet(corner_r)
    )


def _build_front_body():
    """Lime barrel + tapered nose with bore channel, pocket clip on top."""
    barrel = _rounded_rect_prism(BODY_LEN, BODY_W, BODY_H, BODY_CORNER_R)

    # Nose taper: loft from barrel section to smaller nose opening
    nose = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_LEN)
        .rect(BODY_W, BODY_H)
        .workplane(offset=NOSE_LEN)
        .rect(NOSE_OPENING_W, NOSE_OPENING_H)
        .loft(combine=True)
    )
    body = barrel.union(nose)

    # Bore channel through barrel and nose for the nib to slide through
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=BORE_START_X)
        .rect(BORE_W, BORE_H)
        .extrude(BORE_LEN)
        .edges("|X")
        .fillet(0.001)
    )
    body = body.cut(bore)

    # Pocket clip on top, near the rear of the front body
    clip_z = BODY_H / 2.0 + CLIP_THICK / 2.0 - 0.0002
    clip_x0 = 0.006
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(CLIP_W, CLIP_THICK)
        .extrude(CLIP_LEN)
        .edges("|X")
        .fillet(0.0006)
    )
    # Boss anchoring the clip to the barrel at its rear end
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, BODY_H / 2.0 - 0.001)
        .rect(CLIP_W, 0.004)
        .extrude(0.005)
    )
    body = body.union(clip).union(boss)
    return body


def _build_collar_body():
    """Twist collar main body: rounded-rect barrel from X=-COLLAR_LEN to X=0."""
    collar = (
        cq.Workplane("YZ")
        .rect(COLLAR_W, COLLAR_H)
        .extrude(COLLAR_LEN)
        .edges("|X")
        .fillet(COLLAR_CORNER_R)
        .translate((-COLLAR_LEN, 0.0, 0.0))
    )
    # Rear end cap (slightly inset for a finished look)
    end_cap = (
        cq.Workplane("YZ")
        .workplane(offset=-COLLAR_LEN - 0.002)
        .rect(COLLAR_W - 0.001, COLLAR_H - 0.001)
        .extrude(0.002)
        .edges("|X")
        .fillet(COLLAR_CORNER_R - 0.0005)
    )
    collar = collar.union(end_cap)
    return collar


def _build_knurl_band():
    """Raised knurled grip band at the front edge of the twist collar.
    Slightly proud of the collar body, in contrasting dark gray."""
    band_w = COLLAR_W + 2 * KNURL_RAISE
    band_h = COLLAR_H + 2 * KNURL_RAISE
    band_r = COLLAR_CORNER_R + KNURL_RAISE
    band = _rounded_rect_prism(KNURL_LEN, band_w, band_h, band_r)
    band = band.translate((-KNURL_LEN, 0.0, 0.0))
    return band


def _build_knurl_rib(i, n):
    """Single longitudinal grip rib on the knurl band perimeter."""
    angle = 2.0 * math.pi * i / n
    # Position on an ellipse matching the band cross-section
    a = COLLAR_W / 2.0 + KNURL_RAISE * 0.4
    b = COLLAR_H / 2.0 + KNURL_RAISE * 0.4
    cy = a * math.cos(angle)
    cz = b * math.sin(angle)

    protrude = KNURL_RAISE * 1.5
    thin = 0.0005

    # Orient rib: radial protrusion based on which face it's on
    if abs(math.cos(angle)) >= abs(math.sin(angle)):
        # Y-dominant (left or right face): rib protrudes in Y
        rect_w = protrude
        rect_h = thin
    else:
        # Z-dominant (top or bottom face): rib protrudes in Z
        rect_w = thin
        rect_h = protrude

    rib = (
        cq.Workplane("YZ")
        .workplane(offset=-KNURL_LEN)
        .center(cy, cz)
        .rect(rect_w, rect_h)
        .extrude(KNURL_LEN)
    )
    return rib


def _build_nib():
    """Chisel nib: straight holder + tapered wedge to a thin edge.
    Origin at nib base (X=0), extends forward in +X."""
    holder = _rounded_rect_prism(NIB_BASE_LEN, NIB_BASE_W, NIB_BASE_H, 0.0018)
    wedge = (
        cq.Workplane("YZ")
        .workplane(offset=NIB_BASE_LEN)
        .rect(NIB_BASE_W, NIB_BASE_H)
        .workplane(offset=NIB_WEDGE_LEN)
        .rect(NIB_TIP_W, NIB_TIP_H)
        .loft(combine=True)
    )
    return holder.union(wedge)


def _build_felt_tip():
    """Exposed ink-soaked felt sliver at the extreme chisel edge."""
    x0 = NIB_BASE_LEN + NIB_WEDGE_LEN
    felt = (
        cq.Workplane("YZ")
        .workplane(offset=x0 - 0.0015)
        .rect(NIB_TIP_W * 0.96, NIB_TIP_H * 1.05)
        .workplane(offset=0.0025)
        .rect(NIB_TIP_W * 0.9, NIB_TIP_H * 0.8)
        .loft(combine=True)
    )
    return felt


def _build_carrier_stem():
    """Carrier stem with guide ribs for bore-wall contact.
    The stem slides inside the barrel bore; thin guide ribs extend to the
    bore walls to provide centering and physical support."""
    stem = _rounded_rect_prism(CARRIER_STEM_LEN, CARRIER_STEM_W, CARRIER_STEM_H, 0.001)
    stem = stem.translate((-CARRIER_STEM_LEN, 0.0, 0.0))

    # Guide ribs that reach the bore walls (zero-gap contact for connectivity)
    rib_thin = 0.0008  # rib thickness in tangential direction
    # Y-direction ribs (left and right sides)
    y_protrude = (BORE_W - CARRIER_STEM_W) / 2.0
    for sign in (1.0, -1.0):
        y_center = sign * (CARRIER_STEM_W / 2.0 + y_protrude / 2.0)
        y_rib = (
            cq.Workplane("YZ")
            .workplane(offset=-CARRIER_STEM_LEN)
            .center(y_center, 0.0)
            .rect(y_protrude, rib_thin)
            .extrude(CARRIER_STEM_LEN)
        )
        stem = stem.union(y_rib)

    # Z-direction ribs (top and bottom)
    z_protrude = (BORE_H - CARRIER_STEM_H) / 2.0
    for sign in (1.0, -1.0):
        z_center = sign * (CARRIER_STEM_H / 2.0 + z_protrude / 2.0)
        z_rib = (
            cq.Workplane("YZ")
            .workplane(offset=-CARRIER_STEM_LEN)
            .center(0.0, z_center)
            .rect(rib_thin, z_protrude)
            .extrude(CARRIER_STEM_LEN)
        )
        stem = stem.union(z_rib)

    return stem


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twist_marker")

    lime = model.material("lime_body", rgba=LIME)
    dark_lime = model.material("dark_lime", rgba=DARK_LIME)
    black = model.material("black_plastic", rgba=BLACK)
    dark_gray = model.material("dark_grip", rgba=DARK_GRAY)
    felt_mat = model.material("felt_ink", rgba=FELT_COLOR)

    # --- Front body (root): lime barrel + nose + clip ---
    front_body = model.part("front_body")
    front_body.visual(
        mesh_from_cadquery(_build_front_body(), "front_body_shell"),
        material=lime,
        name="barrel_shell",
    )

    # --- Twist collar (rotating rear section) ---
    twist_collar = model.part("twist_collar")
    twist_collar.visual(
        mesh_from_cadquery(_build_collar_body(), "collar_body"),
        material=dark_lime,
        name="collar_body",
    )
    twist_collar.visual(
        mesh_from_cadquery(_build_knurl_band(), "knurl_band"),
        material=dark_gray,
        name="knurl_band",
    )
    # Knurl ribs as repeated sub-parts via loop
    for i in range(N_RIBS):
        twist_collar.visual(
            mesh_from_cadquery(_build_knurl_rib(i, N_RIBS), f"rib_{i}"),
            material=dark_gray,
            name=f"rib_{i}",
        )

    # --- Nib carrier (sliding nib assembly) ---
    nib_carrier = model.part("nib_carrier")
    nib_carrier.visual(
        mesh_from_cadquery(_build_nib(), "nib"),
        material=black,
        name="nib",
    )
    nib_carrier.visual(
        mesh_from_cadquery(_build_felt_tip(), "felt_tip"),
        material=felt_mat,
        name="felt_tip",
    )
    nib_carrier.visual(
        mesh_from_cadquery(_build_carrier_stem(), "carrier_stem"),
        material=black,
        name="carrier_stem",
    )

    # --- Articulations ---
    # Twist collar: REVOLUTE around +X at the seam (rear face of front body)
    model.articulation(
        "body_to_collar",
        ArticulationType.REVOLUTE,
        parent=front_body,
        child=twist_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=TWIST_UPPER,
        ),
    )

    # Nib carrier: PRISMATIC along +X (helical relief couples it to the twist
    # collar rotation; modeled as independent joints since the SDK does not
    # support cross-domain mimic for revolute→prismatic coupling)
    model.articulation(
        "body_to_nib",
        ArticulationType.PRISMATIC,
        parent=front_body,
        child=nib_carrier,
        origin=Origin(xyz=(NIB_RETRACT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.1, lower=0.0, upper=NIB_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_body = object_model.get_part("front_body")
    twist_collar = object_model.get_part("twist_collar")
    nib_carrier = object_model.get_part("nib_carrier")

    twist_joint = object_model.get_articulation("body_to_collar")
    nib_joint = object_model.get_articulation("body_to_nib")

    # --- Joint contract ---
    ctx.check(
        "twist collar is revolute",
        str(twist_joint.joint_type).lower().endswith("revolute"),
        details=f"type={twist_joint.joint_type}",
    )
    ctx.check(
        "nib carrier is prismatic",
        str(nib_joint.joint_type).lower().endswith("prismatic"),
        details=f"type={nib_joint.joint_type}",
    )

    # Twist axis along +X (pen long axis)
    ax = tuple(twist_joint.axis)
    ctx.check(
        "twist axis is along +X",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # Nib prismatic axis along +X
    nax = tuple(nib_joint.axis)
    ctx.check(
        "nib slide axis is along +X",
        abs(nax[0]) > 0.99 and abs(nax[1]) < 1e-6 and abs(nax[2]) < 1e-6,
        details=f"axis={nax}",
    )

    # Nib travel range is positive (helical relief advances nib on twist)
    ctx.check(
        "nib prismatic has positive travel",
        nib_joint.motion_limits is not None
        and nib_joint.motion_limits.upper is not None
        and nib_joint.motion_limits.upper > 0.005,
        details=f"upper={nib_joint.motion_limits.upper if nib_joint.motion_limits else None}",
    )

    # --- Hero geometry ---
    barrel_aabb = ctx.part_element_world_aabb(front_body, elem="barrel_shell")
    ctx.check(
        "front body barrel has substantial length",
        barrel_aabb is not None and (barrel_aabb[1][0] - barrel_aabb[0][0]) > 0.05,
        details=f"aabb={barrel_aabb}",
    )

    collar_aabb = ctx.part_world_aabb(twist_collar)
    ctx.check(
        "twist collar exists behind the seam",
        collar_aabb is not None and collar_aabb[0][0] < -0.020,
        details=f"aabb={collar_aabb}",
    )

    # Knurl band at the seam
    band_aabb = ctx.part_element_world_aabb(twist_collar, elem="knurl_band")
    ctx.check(
        "knurl band reaches the seam plane",
        band_aabb is not None and band_aabb[1][0] >= -0.001,
        details=f"band_aabb={band_aabb}",
    )

    # Knurl ribs present
    rib_count = sum(1 for v in twist_collar.visuals if v.name and v.name.startswith("rib_"))
    ctx.check(
        "knurl band has grip ribs",
        rib_count >= 8,
        details=f"rib_count={rib_count}",
    )

    # Felt tip at the chisel point
    felt_aabb = ctx.part_element_world_aabb(nib_carrier, elem="felt_tip")
    ctx.check(
        "felt tip is at the chisel point",
        felt_aabb is not None and felt_aabb[1][0] >= NIB_RETRACT_X + NIB_TOTAL_LEN - 0.004,
        details=f"felt_aabb={felt_aabb}",
    )

    # --- Retracted pose (q=0 for both joints): nib hidden inside the nose ---
    with ctx.pose({twist_joint: 0.0, nib_joint: 0.0}):
        nib_aabb = ctx.part_element_world_aabb(nib_carrier, elem="nib")
        ctx.check(
            "retracted nib tip is inside the nose",
            nib_aabb is not None and nib_aabb[1][0] < NOSE_TIP_X + 0.001,
            details=f"nib_tip_x={nib_aabb[1][0] if nib_aabb else None}, nose_tip={NOSE_TIP_X:.4f}",
        )
        retracted_tip = nib_aabb[1][0] if nib_aabb else None

    # --- Extended pose (upper limits for both joints): nib protrudes past the nose ---
    with ctx.pose({twist_joint: TWIST_UPPER, nib_joint: NIB_TRAVEL}):
        nib_aabb_ext = ctx.part_element_world_aabb(nib_carrier, elem="nib")
        ctx.check(
            "extended nib protrudes past the nose opening",
            nib_aabb_ext is not None and nib_aabb_ext[1][0] > NOSE_TIP_X + 0.005,
            details=f"nib_tip_x={nib_aabb_ext[1][0] if nib_aabb_ext else None}, nose_tip={NOSE_TIP_X:.4f}",
        )
        extended_tip = nib_aabb_ext[1][0] if nib_aabb_ext else None

    # Nib actually advances when collar is twisted
    ctx.check(
        "twisting advances the nib forward",
        retracted_tip is not None and extended_tip is not None
        and extended_tip > retracted_tip + 0.008,
        details=f"retracted={retracted_tip}, extended={extended_tip}",
    )

    # --- Carrier stem retained insertion ---
    # Stem stays centered in the barrel bore (YZ containment)
    ctx.expect_within(
        nib_carrier,
        front_body,
        axes="yz",
        inner_elem="carrier_stem",
        outer_elem="barrel_shell",
        margin=0.002,
        name="carrier stem stays centered in barrel bore",
    )

    # Stem retains insertion along X at rest
    ctx.expect_overlap(
        nib_carrier,
        front_body,
        axes="x",
        elem_a="carrier_stem",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="carrier stem retains insertion in barrel at rest",
    )

    # --- Knurl band contact at the seam ---
    ctx.expect_contact(
        twist_collar,
        front_body,
        elem_a="collar_body",
        elem_b="barrel_shell",
        contact_tol=0.001,
        name="twist collar meets front body at the seam",
    )

    # --- Overlap allowances ---
    # The nib and stem slide inside the hollow barrel bore (nested prismatic fit)
    ctx.allow_overlap(
        front_body,
        nib_carrier,
        elem_a="barrel_shell",
        elem_b="carrier_stem",
        reason="Carrier stem slides inside the barrel bore channel (nested prismatic fit).",
    )
    ctx.allow_overlap(
        front_body,
        nib_carrier,
        elem_a="barrel_shell",
        elem_b="nib",
        reason="Nib passes through the nose bore during extension/retraction.",
    )
    ctx.allow_overlap(
        front_body,
        nib_carrier,
        elem_a="barrel_shell",
        elem_b="felt_tip",
        reason="Felt tip passes through the nose opening during extension.",
    )

    return ctx.report()


object_model = build_object_model()
