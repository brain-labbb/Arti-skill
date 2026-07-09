from __future__ import annotations

# Retractable click highlighter marker pen (STABILO BOSS variant).
#
# This is a retractable click marker: the lime rounded-rect barrel carries a
# nose cone with a front bore opening, a flat pocket clip, and a click plunger
# button at the rear. The chisel nib + felt tip ride on an internal carrier
# that slides along the pen's long axis (+X) on a PRISMATIC joint.
#
# At q=0 the nib is fully retracted inside the barrel; only the rounded nose
# cone of the barrel is visible at the front. At the upper limit the nib
# extends forward through the nose-cone opening into writing position.
#
# Frame convention:
#   +X = pen length (front of pen at +X, rear at -X)
#   cross-section lies in the Y-Z plane (Y = width, Z = height)

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
BARREL_LEN = 0.110  # lime body length (rear face to nose junction)
BARREL_W = 0.0170   # body width (Y)
BARREL_H = 0.0120   # body height (Z)
BODY_CORNER_R = 0.0030

# Nose cone: tapered front section with a bore for nib passage.
NOSE_LEN = 0.018
NOSE_TIP_W = 0.0140
NOSE_TIP_H = 0.0110
BORE_W = 0.0130     # rectangular bore (slightly larger than nib base)
BORE_H = 0.0100

# Nib (chisel wedge) – same geometry as parent.
NIB_BASE_LEN = 0.0080
NIB_BASE_W = 0.0120
NIB_BASE_H = 0.0090
NIB_WEDGE_LEN = 0.0120
NIB_TIP_W = 0.0090
NIB_TIP_H = 0.0018

# Internal carrier cylinder (the nib cartridge inside the barrel).
CARRIER_R = 0.005
CARRIER_LEN = 0.030

# Click plunger button at rear.
CLICK_LEN = 0.008
CLICK_R = 0.004

# Pocket clip on barrel top.
CLIP_LEN = 0.035
CLIP_W = 0.007
CLIP_THICK = 0.0016

# Prismatic joint: retracted position and travel.
RETRACTED_X = 0.080   # nib base start (X) at q=0
EXTENSION = 0.038     # travel distance (m)

# Derived positions.
NOSE_FRONT_X = BARREL_LEN + NOSE_LEN  # front face of nose cone
NIB_TOTAL_LEN = NIB_BASE_LEN + NIB_WEDGE_LEN  # 0.020 m

# Materials.
LIME = (0.82, 0.93, 0.13, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)
DARK_GRAY = (0.28, 0.28, 0.30, 1.0)  # metal pocket clip
FELT = (0.78, 0.90, 0.15, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rounded_rect_prism(length: float, width: float, height: float, corner_r: float):
    """Rounded-rectangle prism, long axis +X, centered on Y/Z, X in [0, length]."""
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .extrude(length)
        .edges("|X")
        .fillet(corner_r)
    )


def _build_barrel() -> object:
    """Lime barrel body with integrated nose cone and rectangular bore."""
    body = _rounded_rect_prism(BARREL_LEN, BARREL_W, BARREL_H, BODY_CORNER_R)

    # Nose cone: smooth taper from barrel cross-section to smaller tip.
    nose = (
        cq.Workplane("YZ")
        .workplane(offset=BARREL_LEN)
        .rect(BARREL_W, BARREL_H)
        .workplane(offset=NOSE_LEN)
        .rect(NOSE_TIP_W, NOSE_TIP_H)
        .loft()
    )

    # Rectangular bore through nose cone for nib passage.
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=BARREL_LEN)
        .rect(BORE_W, BORE_H)
        .extrude(NOSE_LEN + 0.002)
    )

    result = body.union(nose).cut(bore)
    return result


def _build_nib() -> object:
    """Black chisel nib: holder then tapered wedge to a thin edge.
    Origin at X=0 is the nib base start; nib extends in +X."""
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


def _build_felt_tip() -> object:
    """Exposed ink-soaked felt sliver at the chisel edge."""
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


def _build_carrier_body() -> object:
    """Internal carrier cylinder (the nib cartridge slider).
    Extends from X=-CARRIER_LEN to X=0 in local frame."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=-CARRIER_LEN)
        .circle(CARRIER_R)
        .extrude(CARRIER_LEN)
    )


def _build_pocket_clip() -> object:
    """Flat pocket clip on top of the barrel, anchored near the rear."""
    clip_z = BARREL_H / 2.0 + CLIP_THICK / 2.0 - 0.0002
    clip_x0 = 0.012  # start near rear (after click button)
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(CLIP_W, CLIP_THICK)
        .extrude(CLIP_LEN)
        .edges("|X")
        .fillet(0.0006)
    )
    # Boss connecting clip down to the barrel top at anchor end.
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, BARREL_H / 2.0 - 0.001)
        .rect(CLIP_W, 0.004)
        .extrude(0.005)
    )
    return clip.union(boss)


def _build_click_button() -> object:
    """Click plunger button protruding from the rear face."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=-CLICK_LEN)
        .circle(CLICK_R)
        .extrude(CLICK_LEN)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_click_marker")

    lime = model.material("lime_body", rgba=LIME)
    black = model.material("black_plastic", rgba=BLACK)
    gray = model.material("dark_metal", rgba=DARK_GRAY)
    felt_mat = model.material("felt_ink", rgba=FELT)

    # --- Barrel (root): lime body + nose cone + pocket clip + click button ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel(), "barrel_body"),
        material=lime,
        name="barrel_body",
    )
    barrel.visual(
        mesh_from_cadquery(_build_pocket_clip(), "pocket_clip"),
        material=gray,
        name="pocket_clip",
    )
    barrel.visual(
        mesh_from_cadquery(_build_click_button(), "click_button"),
        material=black,
        name="click_button",
    )

    # --- Nib carrier (child): carrier body + nib + felt tip ---
    # The joint origin at (RETRACTED_X, 0, 0) places the carrier part frame
    # there at q=0. Visuals are authored in the carrier local frame, so
    # origins are at (0, 0, 0) relative to the carrier part frame.
    carrier = model.part("nib_carrier")

    carrier.visual(
        mesh_from_cadquery(_build_carrier_body(), "carrier_body"),
        material=black,
        name="carrier_body",
    )
    carrier.visual(
        mesh_from_cadquery(_build_nib(), "nib"),
        material=black,
        name="nib",
    )
    carrier.visual(
        mesh_from_cadquery(_build_felt_tip(), "felt_tip"),
        material=felt_mat,
        name="felt_tip",
    )

    # --- Prismatic joint: carrier slides along +X inside barrel ---
    model.articulation(
        "barrel_to_carrier",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=carrier,
        origin=Origin(xyz=(RETRACTED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.30, lower=0.0, upper=EXTENSION,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    carrier = object_model.get_part("nib_carrier")
    joint = object_model.get_articulation("barrel_to_carrier")

    # --- Joint contract: prismatic along +X ---
    ctx.check(
        "carrier joint is prismatic",
        str(joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(joint.axis)
    ctx.check(
        "carrier slides along +X",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- Barrel visible features ---
    nose_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
    ctx.check(
        "barrel body extends to nose cone front",
        nose_aabb is not None and nose_aabb[1][0] >= NOSE_FRONT_X - 0.001,
        details=f"barrel_aabb={nose_aabb}",
    )

    clip_aabb = ctx.part_element_world_aabb(barrel, elem="pocket_clip")
    ctx.check(
        "pocket clip present on barrel",
        clip_aabb is not None,
        details=f"clip_aabb={clip_aabb}",
    )

    click_aabb = ctx.part_element_world_aabb(barrel, elem="click_button")
    ctx.check(
        "click plunger button present at rear",
        click_aabb is not None and click_aabb[0][0] < 0.0,
        details=f"click_aabb={click_aabb}",
    )

    # --- Nib and felt tip present ---
    nib_aabb = ctx.part_element_world_aabb(carrier, elem="nib")
    ctx.check(
        "chisel nib present on carrier",
        nib_aabb is not None,
        details=f"nib_aabb={nib_aabb}",
    )

    felt_aabb = ctx.part_element_world_aabb(carrier, elem="felt_tip")
    ctx.check(
        "felt tip present on carrier",
        felt_aabb is not None,
        details=f"felt_aabb={felt_aabb}",
    )

    # --- Carrier centered in barrel on YZ axes ---
    ctx.expect_within(
        carrier,
        barrel,
        axes="yz",
        inner_elem="carrier_body",
        outer_elem="barrel_body",
        margin=0.002,
        name="carrier stays centered in barrel bore",
    )

    # --- Retracted pose (q=0): nib fully behind nose cone opening ---
    nib_tip_x_retracted = RETRACTED_X + NIB_TOTAL_LEN  # ~0.100
    with ctx.pose({joint: 0.0}):
        nib_aabb_0 = ctx.part_element_world_aabb(carrier, elem="nib")
        ctx.check(
            "retracted nib is behind nose cone opening",
            nib_aabb_0 is not None and nib_aabb_0[1][0] < NOSE_FRONT_X - 0.002,
            details=f"nib_front={nib_aabb_0[1][0]:.4f}, nose_front={NOSE_FRONT_X:.4f}",
        )
        retracted_nib_front = nib_aabb_0[1][0] if nib_aabb_0 else None

    # --- Extended pose (upper limit): nib protrudes past nose cone ---
    upper = joint.motion_limits.upper
    nib_tip_x_extended = RETRACTED_X + upper + NIB_TOTAL_LEN
    with ctx.pose({joint: upper}):
        nib_aabb_ext = ctx.part_element_world_aabb(carrier, elem="nib")
        ctx.check(
            "extended nib protrudes past nose cone opening",
            nib_aabb_ext is not None and nib_aabb_ext[1][0] > NOSE_FRONT_X + 0.002,
            details=f"nib_front={nib_aabb_ext[1][0]:.4f}, nose_front={NOSE_FRONT_X:.4f}",
        )
        extended_nib_front = nib_aabb_ext[1][0] if nib_aabb_ext else None

        # Carrier stays centered even when extended.
        ctx.expect_within(
            carrier,
            barrel,
            axes="yz",
            inner_elem="carrier_body",
            outer_elem="barrel_body",
            margin=0.002,
            name="extended carrier stays centered in barrel",
        )

    # --- Nib actually moves forward ---
    ctx.check(
        "nib extends forward when clicked",
        retracted_nib_front is not None
        and extended_nib_front is not None
        and extended_nib_front > retracted_nib_front + 0.02,
        details=f"retracted={retracted_nib_front}, extended={extended_nib_front}",
    )

    # --- The carrier is intentionally nested inside the barrel as a sliding
    # mechanism. It is fully enclosed and may not show surface contact with the
    # barrel body (the barrel is modeled as a solid proxy, not a hollow shell).
    ctx.allow_isolated_part(
        carrier,
        reason="Nib carrier slides inside the barrel body as a nested prismatic mechanism.",
    )

    # --- Intentional overlaps: carrier and nib slide inside solid barrel ---
    ctx.allow_overlap(
        barrel,
        carrier,
        elem_a="barrel_body",
        elem_b="carrier_body",
        reason="Carrier cylinder slides inside the barrel body (nested prismatic fit).",
    )
    ctx.allow_overlap(
        barrel,
        carrier,
        elem_a="barrel_body",
        elem_b="nib",
        reason="Nib retracts inside the barrel body at q=0 (retracted click mechanism).",
    )
    ctx.allow_overlap(
        barrel,
        carrier,
        elem_a="barrel_body",
        elem_b="felt_tip",
        reason="Felt tip retracts inside the barrel body with the nib.",
    )

    # Proof: carrier body retains insertion along X at both rest and extended.
    ctx.expect_overlap(
        carrier,
        barrel,
        axes="x",
        elem_a="carrier_body",
        elem_b="barrel_body",
        min_overlap=0.005,
        name="retracted carrier retains insertion in barrel",
    )
    with ctx.pose({joint: upper}):
        ctx.expect_overlap(
            carrier,
            barrel,
            axes="x",
            elem_a="carrier_body",
            elem_b="barrel_body",
            min_overlap=0.003,
            name="extended carrier retains partial insertion in barrel",
        )

    return ctx.report()


object_model = build_object_model()
