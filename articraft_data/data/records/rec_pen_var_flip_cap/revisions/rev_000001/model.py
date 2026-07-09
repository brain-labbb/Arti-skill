from __future__ import annotations

# STABILO BOSS ORIGINAL highlighter with FLIP CAP.
#
# Variant of the parent pull-off cap model: the prismatic detachable cap is
# replaced with a tethered hinged flip cap that stays attached to the barrel
# by a living hinge at the top of the front collar.  The cap is a REVOLUTE
# joint whose axis is transverse to the pen (along Y, perpendicular to the
# +X length axis), located at a visible hinge knuckle on the collar top.
# At q=0 the cap is closed and covers the chisel nib; increasing q swings
# the cap up and away to expose the nib.
#
# Frame convention:
#   +X = pen length (front at +X, rear at -X)
#   Y  = width, Z = height

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
BARREL_LEN = 0.090       # lime body length (rear to front shoulder)
BARREL_W = 0.0170        # body width (Y)
BARREL_H = 0.0120        # body height (Z)
BODY_CORNER_R = 0.0030

# Front collar (steps down from body, cap registers here)
COLLAR_LEN = 0.0060
COLLAR_W = 0.0150
COLLAR_H = 0.0105
COLLAR_CORNER_R = 0.0028

# Nib (black chisel / wedge)
NIB_BASE_LEN = 0.0080
NIB_BASE_W = 0.0120
NIB_BASE_H = 0.0090
NIB_WEDGE_LEN = 0.0120
NIB_TIP_W = 0.0090
NIB_TIP_H = 0.0018

# Flip cap (hinged at top-rear, opens upward)
CAP_LEN = 0.030          # forward extent from hinge origin
CAP_KNUCKLE = 0.002      # rear extent for hinge knuckle area
CAP_W = 0.0190           # outer width (wider than collar for wrap-around)
CAP_TOP_Z = 0.0030       # height above hinge origin
CAP_DEPTH = 0.0120       # depth below hinge origin
CAP_WALL = 0.0015        # shell wall thickness
CAP_CORNER_R = 0.0030    # outer fillet radius
CAP_CLIP_LEN = 0.022     # pocket clip along cap top

# Hinge knuckle
HINGE_R = 0.0018         # knuckle cylinder radius
HINGE_LEN = 0.008        # knuckle length along Y

# Materials
LIME = (0.82, 0.93, 0.13, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)
FELT = (0.78, 0.90, 0.15, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rounded_rect_prism(length: float, width: float, height: float, corner_r: float):
    """Rounded-rect prism along +X, centered on Y/Z, spanning X in [0, length]."""
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .extrude(length)
        .edges("|X")
        .fillet(corner_r)
    )


def _build_barrel():
    """Lime body + stepped front collar + hinge lug on collar top."""
    body = _rounded_rect_prism(BARREL_LEN, BARREL_W, BARREL_H, BODY_CORNER_R)

    collar = _rounded_rect_prism(
        COLLAR_LEN, COLLAR_W, COLLAR_H, COLLAR_CORNER_R
    ).translate((BARREL_LEN, 0.0, 0.0))
    body = body.union(collar)

    # Hinge lug: a small cylinder along Y sitting on top of the collar at the
    # barrel-collar junction.  Represents the fixed barrel-side hinge anchor.
    lug = (
        cq.Workplane("XZ")
        .workplane(offset=-HINGE_LEN / 2.0)
        .center(BARREL_LEN, COLLAR_H / 2.0)
        .circle(HINGE_R + 0.0003)
        .extrude(HINGE_LEN)
    )
    body = body.union(lug)

    return body


def _build_nib():
    """Black chisel nib: straight holder + tapered wedge to thin edge.
    Origin X=0 corresponds to the front shoulder of the collar."""
    holder = _rounded_rect_prism(NIB_BASE_LEN, NIB_BASE_W, NIB_BASE_H, 0.0018)

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


def _build_felt_tip():
    """Exposed ink-soaked felt sliver at the extreme chisel edge."""
    x0 = NIB_BASE_LEN + NIB_WEDGE_LEN
    return (
        cq.Workplane("YZ")
        .workplane(offset=x0 - 0.0015)
        .rect(NIB_TIP_W * 0.96, NIB_TIP_H * 1.05)
        .workplane(offset=0.0025)
        .rect(NIB_TIP_W * 0.9, NIB_TIP_H * 0.8)
        .loft(combine=True)
    )


def _build_flip_cap():
    """Flip cap: hollow shell hinged at rear-top (local origin = hinge point).
    At q=0 the cap extends +X (forward along pen) and -Z (downward to cover
    the nib).  Open at the rear face so the nib enters when closed."""
    total_x = CAP_LEN + CAP_KNUCKLE
    total_z = CAP_DEPTH + CAP_TOP_Z
    # Center Z of the outer bounding box
    cz = (CAP_TOP_Z - CAP_DEPTH) / 2.0

    # Outer shell: rounded-rect prism from X=-CAP_KNUCKLE to CAP_LEN
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=-CAP_KNUCKLE)
        .center(0.0, cz)
        .rect(CAP_W, total_z)
        .extrude(total_x)
        .edges("|X")
        .fillet(CAP_CORNER_R)
    )

    # Inner cavity: open at the rear face, leaving top / bottom / side / front
    # walls of thickness CAP_WALL.
    cav_w = CAP_W - 2.0 * CAP_WALL
    cav_bottom_z = -CAP_DEPTH + CAP_WALL     # leave bottom wall
    cav_top_z = CAP_TOP_Z - CAP_WALL         # leave top wall
    cav_h = cav_top_z - cav_bottom_z
    cav_cz = (cav_top_z + cav_bottom_z) / 2.0
    # Cavity X extent: from behind the rear face to just before the front wall
    cav_x_start = -CAP_KNUCKLE - 0.002       # ensure full rear cut-through
    cav_x_len = total_x - CAP_WALL + 0.001   # leave front wall only

    cavity = (
        cq.Workplane("YZ")
        .workplane(offset=cav_x_start)
        .center(0.0, cav_cz)
        .rect(cav_w, cav_h)
        .extrude(cav_x_len)
    )
    cap = outer.cut(cavity)

    # Hinge knuckle: small cylinder along Y at the local origin, representing
    # the moving hinge barrel on the cap side.
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=-HINGE_LEN / 2.0)
        .center(0.0, 0.0)
        .circle(HINGE_R)
        .extrude(HINGE_LEN)
    )
    cap = cap.union(knuckle)

    # Pocket clip: thin flat rib running along the cap top surface.
    clip_thick = 0.0014
    clip_w = 0.006
    clip_x0 = CAP_LEN - CAP_CLIP_LEN
    clip_z = CAP_TOP_Z + clip_thick / 2.0 - 0.0002
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(clip_w, clip_thick)
        .extrude(CAP_CLIP_LEN)
        .edges("|X")
        .fillet(0.0005)
    )
    # Connecting boss: ties the clip down to the cap body near its front end.
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=CAP_LEN - 0.004)
        .center(0.0, CAP_TOP_Z - 0.001)
        .rect(clip_w, 0.0035)
        .extrude(0.0035)
    )
    cap = cap.union(clip).union(boss)

    return cap


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilo_boss_flip_cap")

    lime = model.material("lime_body", rgba=LIME)
    black = model.material("black_plastic", rgba=BLACK)
    felt_mat = model.material("felt_ink", rgba=FELT)

    # --- Barrel (root): lime body + collar + hinge lug + nib + felt ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel(), "barrel_body"),
        material=lime,
        name="barrel_body",
    )

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

    # --- Flip cap (hinged child) ---
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_build_flip_cap(), "cap_shell"),
        material=black,
        name="cap_shell",
    )

    # REVOLUTE joint at the hinge point: top of collar, barrel-collar junction.
    # Axis along -Y so positive q rotates +X toward +Z (cap lifts up/away).
    hinge_x = BARREL_LEN
    hinge_z = COLLAR_H / 2.0

    model.articulation(
        "barrel_to_cap",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=cap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=2.4
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
    nib_tip_x = nib_x + NIB_BASE_LEN + NIB_WEDGE_LEN

    # --- Joint contract: revolute with transverse (Y) axis ---
    ctx.check(
        "cap joint is revolute",
        str(joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(joint.axis)
    ctx.check(
        "hinge axis is transverse (Y)",
        abs(ax[0]) < 1e-6 and abs(ax[1]) > 0.99 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # Hinge origin at collar top, near barrel-collar junction
    ho = joint.origin.xyz
    ctx.check(
        "hinge origin at collar top",
        abs(ho[2] - COLLAR_H / 2.0) < 0.003
        and abs(ho[0] - BARREL_LEN) < 0.005,
        details=f"hinge_origin={ho}",
    )

    # --- Hero geometry present ---
    nib_aabb = ctx.part_element_world_aabb(barrel, elem="nib")
    ctx.check(
        "nib protrudes past barrel front",
        nib_aabb is not None and nib_aabb[1][0] > BARREL_LEN + COLLAR_LEN - 1e-6,
        details=f"nib_aabb={nib_aabb}",
    )

    felt_aabb = ctx.part_element_world_aabb(barrel, elem="felt_tip")
    ctx.check(
        "felt tip at chisel point",
        felt_aabb is not None and felt_aabb[1][0] >= nib_tip_x - 0.002,
        details=f"felt_aabb={felt_aabb}",
    )

    # Cap is wider than the barrel (wraps over the collar/nib)
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        cap_w = cap_aabb[1][1] - cap_aabb[0][1]
        ctx.check(
            "cap wider than barrel",
            cap_w >= BARREL_W,
            details=f"cap_w={cap_w:.4f}",
        )

    # --- Closed pose (q=0): cap covers the nib ---
    with ctx.pose({joint: 0.0}):
        cap_closed = ctx.part_world_aabb(cap)
        ctx.check(
            "closed cap front past chisel tip",
            cap_closed is not None and cap_closed[1][0] >= nib_tip_x - 0.001,
            details=f"cap_front={cap_closed[1] if cap_closed else None}, tip={nib_tip_x:.4f}",
        )
        # Cap encloses nib in cross-section (YZ)
        ctx.expect_within(
            barrel, cap,
            axes="yz",
            inner_elem="nib",
            outer_elem="cap_shell",
            margin=0.002,
            name="closed cap encloses nib cross-section",
        )
        seated_top = cap_closed[1][2] if cap_closed else None

    # --- Open pose (upper limit): cap swings up, exposing nib ---
    upper = joint.motion_limits.upper
    with ctx.pose({joint: upper}):
        cap_open = ctx.part_world_aabb(cap)
        # Cap has swung upward: its highest point is well above closed position
        ctx.check(
            "open cap top above closed position",
            cap_open is not None
            and seated_top is not None
            and cap_open[1][2] > seated_top + 0.005,
            details=f"seated_top={seated_top}, open_top={cap_open[1][2] if cap_open else None}",
        )
        # Cap front (which covered the nib) has swung behind the nib:
        # the cap's max-X no longer reaches the nib tip (nib is exposed).
        if cap_open is not None:
            ctx.check(
                "open cap swung away from nib tip",
                cap_open[1][0] < nib_tip_x - 0.002,
                details=f"cap_max_x={cap_open[1][0]:.4f}, nib_tip={nib_tip_x:.4f}",
            )

    # --- Intentional overlaps: flip cap capture fit ---
    ctx.allow_overlap(
        cap, barrel,
        elem_a="cap_shell", elem_b="nib",
        reason="Flip cap encloses the nib when closed (capture fit).",
    )
    ctx.allow_overlap(
        cap, barrel,
        elem_a="cap_shell", elem_b="barrel_body",
        reason="Cap seats over the collar area and hinge knuckle nests in the lug when closed.",
    )
    ctx.allow_overlap(
        cap, barrel,
        elem_a="cap_shell", elem_b="felt_tip",
        reason="Cap encloses the felt tip when closed to prevent drying.",
    )

    return ctx.report()


object_model = build_object_model()
