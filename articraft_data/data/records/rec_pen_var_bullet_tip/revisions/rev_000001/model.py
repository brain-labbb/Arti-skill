from __future__ import annotations

# STABILO BOSS highlighter variant: bullet-tip fiber nib.
#
# This variant replaces the flat chisel wedge nib with a rounded bullet-shaped
# conical fiber tip. The nib tapers from a cylindrical base at the front collar
# to a small rounded writing point on the pen's centerline, with an exposed
# ink-soaked fiber sliver at the very point. The lime rounded-rect barrel,
# front collar, flat pocket clip, and pull-off prismatic cap remain unchanged.
#
# The cap is the moving part: it pulls straight off the front of the barrel
# (a push/pull friction fit), so it is modeled as a PRISMATIC joint along the
# pen's long axis (+X). At q=0 the cap is fully seated over the nib (closed);
# positive q draws it forward off the body.
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
BARREL_LEN = 0.090  # length of the lime body (rear shoulder to front shoulder)
BARREL_W = 0.0170  # body width (Y)
BARREL_H = 0.0120  # body height (Z)
BODY_CORNER_R = 0.0030  # rounded corners of the rectangular section

# Front shoulder: a short stepped collar where the cap registers.
COLLAR_LEN = 0.0060
COLLAR_W = 0.0150
COLLAR_H = 0.0105
COLLAR_CORNER_R = 0.0028

# Nib (black bullet / conical fiber tip) protruding forward from the collar.
NIB_BASE_LEN = 0.0080  # cylindrical holder right after the collar
NIB_BASE_R = 0.0050  # radius of the cylindrical base (diameter 0.010)
NIB_TAPER_LEN = 0.0120  # tapered conical section
NIB_TIP_R = 0.0012  # radius of the rounded bullet tip

# Cap (black, hollow, rounded-rect, closed at its front, open at its rear).
CAP_LEN = 0.0420
CAP_OUTER_W = 0.0182
CAP_OUTER_H = 0.0132
CAP_WALL = 0.0014
CAP_CORNER_R = 0.0034
CAP_CLIP_LEN = 0.0260  # pocket-clip flat along the cap top

# Cap seats so its rear lip overlaps the front of the lime barrel.
CAP_SEAT_OVERLAP = 0.0060  # how far the cap mouth slides back onto the barrel

# Materials
LIME = (0.82, 0.93, 0.13, 1.0)  # fluorescent yellow-green body
BLACK = (0.07, 0.07, 0.08, 1.0)  # nib + cap
FELT = (0.12, 0.12, 0.13, 1.0)  # exposed ink-soaked fiber at the very tip


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rounded_rect_prism(length: float, width: float, height: float, corner_r: float):
    """A rounded-rectangle prism whose long axis is +X, centered on Y/Z,
    spanning X in [0, length]."""
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .extrude(length)
        .edges("|X")
        .fillet(corner_r)
    )


def _build_barrel() -> object:
    """Lime body + stepped front collar, as one solid (the pen body)."""
    body = _rounded_rect_prism(BARREL_LEN, BARREL_W, BARREL_H, BODY_CORNER_R)

    # Front collar steps down from the body so the cap mouth seats over it.
    collar = _rounded_rect_prism(
        COLLAR_LEN, COLLAR_W, COLLAR_H, COLLAR_CORNER_R
    ).translate((BARREL_LEN, 0.0, 0.0))

    body = body.union(collar)
    return body


def _build_nib() -> object:
    """Black bullet nib: a cylindrical base then a tapered cone to a rounded
    writing point on the pen centerline.
    Origin at X=0 corresponds to the front shoulder of the barrel collar."""
    base_r = NIB_BASE_R
    tip_r = NIB_TIP_R

    # Cylindrical base section (fits inside the collar opening).
    base = cq.Workplane("YZ").circle(base_r).extrude(NIB_BASE_LEN)

    # Tapered conical section: loft from the base radius to the tip radius.
    taper = (
        cq.Workplane("YZ")
        .workplane(offset=NIB_BASE_LEN)
        .circle(base_r)
        .workplane(offset=NIB_TAPER_LEN)
        .circle(tip_r)
        .loft()
    )

    # Rounded bullet tip: a small sphere capping the cone.
    tip_x = NIB_BASE_LEN + NIB_TAPER_LEN
    tip_sphere = cq.Workplane("YZ").workplane(offset=tip_x).sphere(tip_r)

    return base.union(taper).union(tip_sphere)


def _build_felt_tip() -> object:
    """The exposed ink-soaked fiber sliver at the extreme bullet point."""
    tip_x = NIB_BASE_LEN + NIB_TAPER_LEN + NIB_TIP_R * 0.4
    felt_r = 0.0006
    felt_len = 0.0018
    felt = (
        cq.Workplane("YZ")
        .workplane(offset=tip_x)
        .circle(felt_r)
        .extrude(felt_len)
    )
    return felt


def _build_cap() -> object:
    """Hollow black cap, closed at its front (+X), open at its rear (-X) so it
    slides over the nib. Authored in its own frame: rear mouth at X=0, front
    closed end at X=CAP_LEN. Includes a flat pocket clip on top."""
    outer = _rounded_rect_prism(CAP_LEN, CAP_OUTER_W, CAP_OUTER_H, CAP_CORNER_R)

    # Hollow it out from the rear mouth, leaving the front end closed.
    bore_w = CAP_OUTER_W - 2 * CAP_WALL
    bore_h = CAP_OUTER_H - 2 * CAP_WALL
    bore_len = CAP_LEN - CAP_WALL  # leave a front wall of thickness CAP_WALL
    bore = (
        cq.Workplane("YZ")
        .rect(bore_w, bore_h)
        .extrude(bore_len)
        .edges("|X")
        .fillet(CAP_CORNER_R - CAP_WALL)
    )
    cap = outer.cut(bore)

    # Pocket clip: a thin flat rib running along the top, standing slightly
    # proud and anchored to the closed front end of the cap.
    clip_thick = 0.0016
    clip_w = 0.0070
    clip_x0 = CAP_LEN - CAP_CLIP_LEN
    clip_z = CAP_OUTER_H / 2.0 + clip_thick / 2.0 - 0.0002
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(clip_w, clip_thick)
        .extrude(CAP_CLIP_LEN)
        .edges("|X")
        .fillet(0.0006)
    )
    # Small connecting boss tying the clip down to the cap body at its front.
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=CAP_LEN - 0.0050)
        .center(0.0, CAP_OUTER_H / 2.0 - 0.0010)
        .rect(clip_w, 0.0040)
        .extrude(0.0045)
    )
    cap = cap.union(clip).union(boss)
    return cap


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilo_boss_highlighter")

    lime = model.material("lime_body", rgba=LIME)
    black = model.material("black_plastic", rgba=BLACK)
    felt_mat = model.material("felt_ink", rgba=FELT)

    # --- Pen body (root): lime barrel + collar, plus the black nib up front ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel(), "barrel_body"),
        material=lime,
        name="barrel_body",
    )
    # Nib visuals live on the barrel part (rigidly part of the pen body), shifted
    # forward to start at the front shoulder of the collar.
    nib_x = BARREL_LEN + COLLAR_LEN
    barrel.visual(
        mesh_from_cadquery(_build_nib(), "nib"),
        origin=Origin(xyz=(nib_x, 0.0, 0.0)),
        material=black,
        name="nib",
    )
    barrel.visual(
        mesh_from_cadquery(_build_felt_tip(), "felt_tip"),
        origin=Origin(xyz=(nib_x, 0.0, 0.0)),
        material=felt_mat,
        name="felt_tip",
    )

    # --- Cap (moving part): hollow black shell that pulls off forward ---
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_build_cap(), "cap_shell"),
        material=black,
        name="cap_shell",
    )

    # Seated cap pose: cap rear mouth sits back over the front of the barrel by
    # CAP_SEAT_OVERLAP, so the cap front fully covers the nib.
    # Cap is authored with its mouth at local X=0; place the joint origin so that
    # at q=0 the mouth is at (BARREL_LEN - CAP_SEAT_OVERLAP).
    seat_x = BARREL_LEN - CAP_SEAT_OVERLAP
    # Travel needed to fully clear the nib: past the tip plus a margin.
    nib_tip_x = nib_x + NIB_BASE_LEN + NIB_TAPER_LEN + NIB_TIP_R
    full_clear = (nib_tip_x - seat_x) + 0.006

    model.articulation(
        "barrel_to_cap",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=cap,
        origin=Origin(xyz=(seat_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.25, lower=0.0, upper=full_clear
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("barrel_to_cap")

    nib_x = BARREL_LEN + COLLAR_LEN
    nib_tip_x = nib_x + NIB_BASE_LEN + NIB_TAPER_LEN + NIB_TIP_R

    # --- Joint contract: prismatic along the pen's long axis (+X) ---
    ctx.check(
        "cap joint is prismatic",
        str(joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(joint.axis)
    ctx.check(
        "cap slides along +X",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- Hero parts present and placed ---
    # Nib lives at the front of the pen, ahead of the barrel body.
    nib_aabb = ctx.part_element_world_aabb(barrel, elem="nib")
    ctx.check(
        "black nib protrudes past the barrel front",
        nib_aabb is not None and nib_aabb[1][0] > BARREL_LEN + COLLAR_LEN - 1e-6,
        details=f"nib_aabb={nib_aabb}",
    )
    # Verify the bullet nib is roughly circular in cross-section (not rectangular
    # like the chisel variant). The Y and Z dimensions should be similar.
    if nib_aabb is not None:
        nib_w = nib_aabb[1][1] - nib_aabb[0][1]
        nib_h = nib_aabb[1][2] - nib_aabb[0][2]
        ctx.check(
            "bullet nib has circular cross-section (Y ≈ Z)",
            abs(nib_w - nib_h) / max(nib_w, nib_h) < 0.15,
            details=f"nib_w={nib_w:.4f}, nib_h={nib_h:.4f}",
        )
    # Exposed felt tip is at the extreme front of the pen.
    felt_aabb = ctx.part_element_world_aabb(barrel, elem="felt_tip")
    ctx.check(
        "fiber tip is at the bullet point",
        felt_aabb is not None and felt_aabb[1][0] >= nib_tip_x - 0.002,
        details=f"felt_aabb={felt_aabb}",
    )
    # Verify the fiber tip is small and round (bullet style), not wide and flat
    # like the chisel variant.
    if felt_aabb is not None:
        felt_w = felt_aabb[1][1] - felt_aabb[0][1]
        felt_h = felt_aabb[1][2] - felt_aabb[0][2]
        ctx.check(
            "fiber tip is small and round (not wide chisel)",
            felt_w < 0.002 and felt_h < 0.002,
            details=f"felt_w={felt_w:.4f}, felt_h={felt_h:.4f}",
        )

    # Cap is a roughly rectangular body wider than the barrel (it wraps it).
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        cap_w = cap_aabb[1][1] - cap_aabb[0][1]
        cap_h = cap_aabb[1][2] - cap_aabb[0][2]
        ctx.check(
            "cap is a chunky rectangular shell",
            cap_w >= BARREL_W and cap_h >= BARREL_H,
            details=f"cap_w={cap_w:.4f}, cap_h={cap_h:.4f}",
        )

    # --- Closed pose (q=0): cap fully covers / encloses the nib ---
    with ctx.pose({joint: 0.0}):
        # Cap projected footprint surrounds the nib in cross-section.
        ctx.expect_within(
            barrel,
            cap,
            axes="yz",
            inner_elem="nib",
            outer_elem="cap_shell",
            margin=0.001,
            name="seated cap encloses the nib cross-section",
        )
        # Cap front reaches beyond the bullet tip (tip is capped, not exposed).
        cap_closed = ctx.part_world_aabb(cap)
        ctx.check(
            "seated cap front covers the bullet tip",
            cap_closed is not None and cap_closed[1][0] >= nib_tip_x - 1e-4,
            details=f"cap_front={None if cap_closed is None else cap_closed[1][0]:.4f}, tip={nib_tip_x:.4f}",
        )
        seated_front = None if cap_closed is None else cap_closed[1][0]

    # --- Open pose (upper limit): cap pulls forward and clears the nib ---
    upper = joint.motion_limits.upper
    with ctx.pose({joint: upper}):
        cap_open = ctx.part_world_aabb(cap)
        # Cap mouth (its rear / min X) has moved forward past the bullet tip,
        # so the nib is fully exposed.
        ctx.check(
            "pulled cap clears the nib (mouth past the tip)",
            cap_open is not None and cap_open[0][0] >= nib_tip_x - 1e-3,
            details=f"cap_mouth={None if cap_open is None else cap_open[0][0]:.4f}, tip={nib_tip_x:.4f}",
        )
        # And it travelled forward relative to the seated pose.
        open_front = None if cap_open is None else cap_open[1][0]
        ctx.check(
            "cap moves forward when pulled off",
            seated_front is not None
            and open_front is not None
            and open_front > seated_front + 0.02,
            details=f"seated_front={seated_front}, open_front={open_front}",
        )

    # The seated cap nests over the barrel/nib front: a genuine capture fit.
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="nib",
        reason="Seated cap is a friction fit that intentionally encloses the nib (capture fit).",
    )
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="barrel_body",
        reason="Cap mouth slides back over the front collar of the barrel to seat (push-on fit).",
    )
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="felt_tip",
        reason="Seated cap encloses the felt tip to keep the highlighter from drying out.",
    )

    return ctx.report()


object_model = build_object_model()
