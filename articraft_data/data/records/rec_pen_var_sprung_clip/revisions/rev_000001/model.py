from __future__ import annotations

# STABILO BOSS ORIGINAL highlighter.
#
# The reference image shows a thick rectangular-barrel highlighter: a lime /
# fluorescent-yellow body with rounded corners and the "STABILO BOSS ORIGINAL"
# branding, a black chisel (wedge) nib at the front, and a separate black cap.
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

# Nib (black wedge / chisel point) protruding forward from the collar.
NIB_BASE_LEN = 0.0080  # straight black holder right after the collar
NIB_BASE_W = 0.0120
NIB_BASE_H = 0.0090
NIB_WEDGE_LEN = 0.0120  # tapered chisel section
NIB_TIP_W = 0.0090
NIB_TIP_H = 0.0018  # thin chisel edge

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
FELT = (0.78, 0.90, 0.15, 1.0)  # exposed ink-soaked felt at the very tip
CLIP_STEEL = (0.70, 0.71, 0.73, 1.0)  # spring-steel pocket clip


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
    """Black chisel nib: a straight holder then a tapered wedge to a thin edge.
    Origin at X=0 corresponds to the front shoulder of the barrel collar."""
    holder = _rounded_rect_prism(NIB_BASE_LEN, NIB_BASE_W, NIB_BASE_H, 0.0018)

    # Tapered chisel: loft from the holder face to a thin wide edge.
    x0 = NIB_BASE_LEN
    wedge = (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .rect(NIB_BASE_W, NIB_BASE_H)
        .workplane(offset=NIB_WEDGE_LEN)
        .rect(NIB_TIP_W, NIB_TIP_H)
        .loft(combine=True)
    )
    return holder.union(wedge)


def _build_felt_tip() -> object:
    """The exposed ink-soaked felt sliver at the extreme chisel edge."""
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


def _build_cap() -> object:
    """Hollow black cap, closed at its front (+X), open at its rear (-X) so it
    slides over the nib. Authored in its own frame: rear mouth at X=0, front
    closed end at X=CAP_LEN. The separate spring-loaded pocket clip is its own
    part hinged on the cap top near the closed front."""
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

    # Small transverse hinge boss on the cap top near the closed front end.
    # This is the anchor point where the pocket clip pivots. The boss is
    # embedded slightly into the cap shell for a clean boolean union.
    boss_dx = 0.0038  # length along X
    boss_dy = 0.0090  # width across Y
    boss_dz = 0.0022  # height above cap top
    boss_x = CAP_LEN - 0.0040  # center X near front
    boss_z = CAP_OUTER_H / 2.0 + boss_dz / 2.0 - 0.0004  # slight embed
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=boss_x - boss_dx / 2.0)
        .center(0.0, boss_z)
        .rect(boss_dy, boss_dz)
        .extrude(boss_dx)
        .edges("|X")
        .fillet(0.0006)
    )
    cap = cap.union(boss)
    return cap


def _build_clip() -> object:
    """Spring-loaded pocket clip, authored with hinge pivot at the origin.
    At q=0 the strip lies flat along -X (extending backward along the cap top).
    The free tail at the far -X end has a contoured grip lip."""
    L = CAP_CLIP_LEN  # 0.026 m total strip length
    W = 0.0065  # strip width
    T = 0.0012  # strip thickness
    # In clip local frame: origin = hinge pivot. The strip sits with its
    # vertical center at Z=0, so bottom at Z=-T/2 and top at Z=+T/2.

    # Main flat strip: from X = -(L - 0.005) to X = +0.001 (slight overshoot
    # to merge cleanly with the knuckle at origin).
    strip_start = -(L - 0.005)
    strip_end = 0.001
    strip = (
        cq.Workplane("YZ")
        .workplane(offset=strip_start)
        .center(0.0, 0.0)
        .rect(W, T)
        .extrude(strip_end - strip_start)
        .edges("|X")
        .fillet(0.0003)
    )

    # Contoured tail: the free end ramps up slightly to form a grip lip.
    # Loft from a lifted section at the very tip back down to the flat strip.
    tail = (
        cq.Workplane("YZ")
        .workplane(offset=-L)
        .center(0.0, 0.0012)  # tip lifted ~1.2 mm above strip center
        .rect(W * 0.82, T * 0.85)
        .workplane(offset=0.006)
        .center(0.0, -0.0012)  # blend back to strip center Z=0
        .rect(W, T)
        .loft()
    )

    # Hinge knuckle: a small transverse cylinder at the pivot (origin).
    kr = 0.0011  # knuckle radius
    kw = W * 0.38  # half-width of knuckle along Y
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=-kw)
        .center(0.0, 0.0)
        .circle(kr)
        .extrude(kw * 2.0)
    )

    return strip.union(tail).union(knuckle)


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
    nib_tip_x = nib_x + NIB_BASE_LEN + NIB_WEDGE_LEN
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

    # --- Pocket clip: separate spring-loaded part hinged on the cap top ---
    clip_mat = model.material("clip_steel", rgba=CLIP_STEEL)
    clip = model.part("clip")
    clip.visual(
        mesh_from_cadquery(_build_clip(), "clip_body"),
        material=clip_mat,
        name="clip_body",
    )

    # Hinge axis: transverse to the pen (along Y), at the anchored front end
    # of the clip on the cap top. The pivot sits at the clip strip vertical
    # center so the strip lies flat on the cap at q=0 and positive q lifts the
    # free tail upward away from the cap surface.
    hinge_x = CAP_LEN - 0.0040  # cap-local X, near the closed front
    hinge_z = CAP_OUTER_H / 2.0 + 0.0012  # pivot at strip center height
    model.articulation(
        "cap_to_clip",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=clip,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=0.55
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
    clip = object_model.get_part("clip")
    cap_joint = object_model.get_articulation("barrel_to_cap")
    clip_joint = object_model.get_articulation("cap_to_clip")

    nib_x = BARREL_LEN + COLLAR_LEN
    nib_tip_x = nib_x + NIB_BASE_LEN + NIB_WEDGE_LEN

    # --- Cap joint contract: prismatic along the pen's long axis (+X) ---
    ctx.check(
        "cap joint is prismatic",
        str(cap_joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={cap_joint.joint_type}",
    )
    cap_ax = tuple(cap_joint.axis)
    ctx.check(
        "cap slides along +X",
        abs(cap_ax[0]) > 0.99 and abs(cap_ax[1]) < 1e-6 and abs(cap_ax[2]) < 1e-6,
        details=f"axis={cap_ax}",
    )

    # --- Clip joint contract: revolute, axis transverse to pen (Y) ---
    ctx.check(
        "clip joint is revolute",
        str(clip_joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={clip_joint.joint_type}",
    )
    clip_ax = tuple(clip_joint.axis)
    ctx.check(
        "clip hinge axis is transverse to pen (Y)",
        abs(clip_ax[0]) < 1e-6 and abs(clip_ax[1]) > 0.99 and abs(clip_ax[2]) < 1e-6,
        details=f"axis={clip_ax}",
    )

    # --- Hero parts present and placed ---
    # Nib lives at the front of the pen, ahead of the barrel body.
    nib_aabb = ctx.part_element_world_aabb(barrel, elem="nib")
    ctx.check(
        "black nib protrudes past the barrel front",
        nib_aabb is not None and nib_aabb[1][0] > BARREL_LEN + COLLAR_LEN - 1e-6,
        details=f"nib_aabb={nib_aabb}",
    )
    # Exposed felt tip is at the extreme front of the pen.
    felt_aabb = ctx.part_element_world_aabb(barrel, elem="felt_tip")
    ctx.check(
        "felt ink tip is at the chisel point",
        felt_aabb is not None and felt_aabb[1][0] >= nib_tip_x - 0.002,
        details=f"felt_aabb={felt_aabb}",
    )

    # Cap is a rectangular shell wider than the barrel.
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        cap_w = cap_aabb[1][1] - cap_aabb[0][1]
        cap_h = cap_aabb[1][2] - cap_aabb[0][2]
        ctx.check(
            "cap is a chunky rectangular shell",
            cap_w >= BARREL_W and cap_h >= BARREL_H,
            details=f"cap_w={cap_w:.4f}, cap_h={cap_h:.4f}",
        )

    # Clip exists and has a visible body.
    clip_aabb = ctx.part_world_aabb(clip)
    ctx.check(
        "pocket clip has visible geometry",
        clip_aabb is not None and (clip_aabb[1][0] - clip_aabb[0][0]) > 0.010,
        details=f"clip_aabb={clip_aabb}",
    )

    # --- Closed cap pose (q=0): cap fully covers / encloses the nib ---
    with ctx.pose({cap_joint: 0.0}):
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
        # Cap front reaches beyond the chisel tip (tip is capped, not exposed).
        cap_closed = ctx.part_world_aabb(cap)
        ctx.check(
            "seated cap front covers the chisel tip",
            cap_closed is not None and cap_closed[1][0] >= nib_tip_x - 1e-4,
            details=f"cap_front={None if cap_closed is None else cap_closed[1][0]:.4f}, tip={nib_tip_x:.4f}",
        )
        seated_front = None if cap_closed is None else cap_closed[1][0]

    # --- Open cap pose (upper limit): cap pulls forward and clears the nib ---
    cap_upper = cap_joint.motion_limits.upper
    with ctx.pose({cap_joint: cap_upper}):
        cap_open = ctx.part_world_aabb(cap)
        ctx.check(
            "pulled cap clears the nib (mouth past the tip)",
            cap_open is not None and cap_open[0][0] >= nib_tip_x - 1e-3,
            details=f"cap_mouth={None if cap_open is None else cap_open[0][0]:.4f}, tip={nib_tip_x:.4f}",
        )
        open_front = None if cap_open is None else cap_open[1][0]
        ctx.check(
            "cap moves forward when pulled off",
            seated_front is not None
            and open_front is not None
            and open_front > seated_front + 0.02,
            details=f"seated_front={seated_front}, open_front={open_front}",
        )

    # --- Clip hinge: q=0 lies flat, positive q lifts the tail ---
    with ctx.pose({clip_joint: 0.0}):
        clip_rest = ctx.part_world_aabb(clip)
        clip_rest_top = None if clip_rest is None else clip_rest[1][2]
        cap_rest_top = None
        cap_rest = ctx.part_world_aabb(cap)
        if cap_rest is not None:
            cap_rest_top = cap_rest[1][2]
        # At rest, the clip top is close to the cap top (clip lies flat on cap).
        ctx.check(
            "clip lies flat on cap top at rest",
            clip_rest_top is not None
            and cap_rest_top is not None
            and abs(clip_rest_top - cap_rest_top) < 0.006,
            details=f"clip_top={clip_rest_top}, cap_top={cap_rest_top}",
        )

    clip_upper = clip_joint.motion_limits.upper
    with ctx.pose({clip_joint: clip_upper}):
        clip_open = ctx.part_world_aabb(clip)
        clip_open_top = None if clip_open is None else clip_open[1][2]
        # At max opening, the clip free tail lifts well above the cap top.
        ctx.check(
            "clip tail lifts away from cap when opened",
            clip_open_top is not None
            and clip_rest_top is not None
            and clip_open_top > clip_rest_top + 0.003,
            details=f"rest_top={clip_rest_top}, open_top={clip_open_top}",
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
    # The clip knuckle sits at the hinge pivot embedded in the cap boss — a
    # small local overlap at the pivot that represents the hinge barrel.
    ctx.allow_overlap(
        clip,
        cap,
        elem_a="clip_body",
        elem_b="cap_shell",
        reason="Clip hinge knuckle is embedded in the cap hinge boss to form the pivot barrel.",
    )

    return ctx.report()


object_model = build_object_model()
