from __future__ import annotations

# Round-barrel marker pen — variant of the STABILO-style pen family.
#
# Same structure as the parent (barrel + collar + chisel nib + felt tip + cap)
# but with circular cross-sections throughout: barrel, collar, and cap are all
# round cylinders instead of rounded-rectangular prisms.  The pen reads as a
# slim round marker rather than a flat highlighter.
#
# The cap is the moving part: it pulls straight off the front of the barrel
# (a push/pull friction fit), modeled as a PRISMATIC joint along the pen's
# long axis (+X).  At q=0 the cap is fully seated over the nib (closed);
# positive q draws it forward off the body.
#
# Frame convention:
#   +X = pen length (front of pen at +X, rear at -X)
#   cross-section lies in the Y-Z plane

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
BARREL_LEN = 0.090        # length of the body
BARREL_R = 0.0050         # barrel radius (10 mm diameter slim marker)

# Front shoulder: stepped collar where the cap registers.
COLLAR_LEN = 0.006
COLLAR_R = 0.0044         # steps down from barrel

# Nib (black chisel / wedge point) protruding forward from the collar.
NIB_BASE_LEN = 0.008      # straight cylindrical holder right after the collar
NIB_BASE_R = 0.0040       # matches collar scale
NIB_WEDGE_LEN = 0.012     # tapered chisel section
NIB_TIP_W = 0.009         # wide chisel edge
NIB_TIP_H = 0.0018        # thin chisel edge

# Cap (black, hollow cylinder, closed at front, open at rear).
CAP_LEN = 0.042
CAP_OUTER_R = 0.0063      # outer radius
CAP_WALL = 0.0011         # wall thickness
# bore radius = CAP_OUTER_R - CAP_WALL = 0.0052 > BARREL_R, so the cap
# can physically slide over the barrel with 0.2 mm radial clearance.
CAP_CLIP_LEN = 0.026      # pocket-clip flat along the cap top

# Cap seats so its rear mouth overlaps the front of the barrel.
CAP_SEAT_OVERLAP = 0.006  # how far the cap mouth slides back onto the barrel

# Materials
BLUE_BODY = (0.18, 0.45, 0.78, 1.0)   # cobalt-blue barrel
BLACK = (0.07, 0.07, 0.08, 1.0)       # nib + cap
FELT_BLUE = (0.12, 0.35, 0.65, 1.0)   # ink-soaked felt at the chisel edge


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _cylinder(radius: float, length: float, x_offset: float = 0.0):
    """Round cylinder along +X, starting at x_offset."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_offset)
        .circle(radius)
        .extrude(length)
    )


def _build_barrel():
    """Blue cylindrical body + stepped front collar, as one solid."""
    body = _cylinder(BARREL_R, BARREL_LEN)
    collar = _cylinder(COLLAR_R, COLLAR_LEN, x_offset=BARREL_LEN)
    # Small rear end-cap disk to finish the back of the pen.
    rear = _cylinder(BARREL_R, 0.001, x_offset=-0.001)
    return body.union(collar).union(rear)


def _build_nib():
    """Black chisel nib: cylindrical holder transitioning to a flat chisel wedge.
    Origin at X=0 corresponds to the front shoulder of the barrel collar."""
    holder = _cylinder(NIB_BASE_R, NIB_BASE_LEN)

    # Wedge loft: start from a square matching the holder diameter, taper
    # to the thin chisel edge.  Union with the round holder gives a natural
    # round-to-chisel transition.
    x0 = NIB_BASE_LEN
    start_side = NIB_BASE_R * 2.0  # matches holder diameter
    wedge = (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .rect(start_side, start_side)
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


def _build_cap():
    """Hollow black cylindrical cap, closed at its front (+X), open at its
    rear (-X) so it slides over the nib.  Authored in its own frame: rear
    mouth at X=0, front closed end at X=CAP_LEN.  Includes a flat pocket
    clip on top."""
    outer = _cylinder(CAP_OUTER_R, CAP_LEN)

    # Hollow from the rear mouth, leaving a front wall of CAP_WALL thickness.
    bore_r = CAP_OUTER_R - CAP_WALL
    bore_len = CAP_LEN - CAP_WALL
    bore = _cylinder(bore_r, bore_len)
    cap = outer.cut(bore)

    # Pocket clip: thin flat rib running along the top (+Z), anchored to the
    # closed front end of the cap.
    clip_thick = 0.0016
    clip_w = 0.0050
    clip_x0 = CAP_LEN - CAP_CLIP_LEN
    clip_z = CAP_OUTER_R + clip_thick / 2.0 - 0.0002
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(clip_w, clip_thick)
        .extrude(CAP_CLIP_LEN)
        .edges("|X")
        .fillet(0.0005)
    )
    # Connecting boss tying the clip down to the cap body at its front.
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=CAP_LEN - 0.0050)
        .center(0.0, CAP_OUTER_R - 0.0010)
        .rect(clip_w, 0.0040)
        .extrude(0.0045)
    )
    cap = cap.union(clip).union(boss)
    return cap


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_marker_pen")

    blue = model.material("blue_body", rgba=BLUE_BODY)
    black = model.material("black_plastic", rgba=BLACK)
    felt_mat = model.material("felt_ink", rgba=FELT_BLUE)

    # --- Pen body (root): blue barrel + collar, plus the black nib up front ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel(), "barrel_body"),
        material=blue,
        name="barrel_body",
    )
    # Nib visuals live on the barrel part (rigidly part of the pen body),
    # shifted forward to start at the front shoulder of the collar.
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

    # Seated cap pose: cap rear mouth sits back over the front of the barrel
    # by CAP_SEAT_OVERLAP, so the cap front fully covers the nib.
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

    # --- Round cross-section checks ---
    barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
    if barrel_aabb is not None:
        dy = barrel_aabb[1][1] - barrel_aabb[0][1]
        dz = barrel_aabb[1][2] - barrel_aabb[0][2]
        ctx.check(
            "barrel has round cross-section (Y extent matches Z extent)",
            abs(dy - dz) < 0.002,
            details=f"dy={dy:.4f}, dz={dz:.4f}",
        )

    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        cap_dy = cap_aabb[1][1] - cap_aabb[0][1]
        cap_dz = cap_aabb[1][2] - cap_aabb[0][2]
        # The clip adds height, so allow a larger tolerance for the cap.
        ctx.check(
            "cap body is approximately round (Y and Z extents similar)",
            cap_dy > 0.0 and cap_dz > 0.0 and min(cap_dy, cap_dz) / max(cap_dy, cap_dz) > 0.65,
            details=f"cap_dy={cap_dy:.4f}, cap_dz={cap_dz:.4f}",
        )

    # --- Hero parts present and placed ---
    nib_aabb = ctx.part_element_world_aabb(barrel, elem="nib")
    ctx.check(
        "black nib protrudes past the barrel front",
        nib_aabb is not None and nib_aabb[1][0] > BARREL_LEN + COLLAR_LEN - 1e-6,
        details=f"nib_aabb={nib_aabb}",
    )

    felt_aabb = ctx.part_element_world_aabb(barrel, elem="felt_tip")
    ctx.check(
        "felt ink tip is at the chisel point",
        felt_aabb is not None and felt_aabb[1][0] >= nib_tip_x - 0.002,
        details=f"felt_aabb={felt_aabb}",
    )

    # --- Closed pose (q=0): cap fully covers / encloses the nib ---
    with ctx.pose({joint: 0.0}):
        ctx.expect_within(
            barrel,
            cap,
            axes="yz",
            inner_elem="nib",
            outer_elem="cap_shell",
            margin=0.001,
            name="seated cap encloses the nib cross-section",
        )
        cap_closed = ctx.part_world_aabb(cap)
        ctx.check(
            "seated cap front covers the chisel tip",
            cap_closed is not None and cap_closed[1][0] >= nib_tip_x - 1e-4,
            details=(
                f"cap_front={None if cap_closed is None else cap_closed[1][0]:.4f}, "
                f"tip={nib_tip_x:.4f}"
            ),
        )
        seated_front = None if cap_closed is None else cap_closed[1][0]

    # --- Open pose (upper limit): cap pulls forward and clears the nib ---
    upper = joint.motion_limits.upper
    with ctx.pose({joint: upper}):
        cap_open = ctx.part_world_aabb(cap)
        ctx.check(
            "pulled cap clears the nib (mouth past the tip)",
            cap_open is not None and cap_open[0][0] >= nib_tip_x - 1e-3,
            details=(
                f"cap_mouth={None if cap_open is None else cap_open[0][0]:.4f}, "
                f"tip={nib_tip_x:.4f}"
            ),
        )
        open_front = None if cap_open is None else cap_open[1][0]
        ctx.check(
            "cap moves forward when pulled off",
            seated_front is not None
            and open_front is not None
            and open_front > seated_front + 0.02,
            details=f"seated_front={seated_front}, open_front={open_front}",
        )

    # The seated cap nests over the barrel/nib front: a genuine capture fit.
    # With bore_r > barrel_r the barrel slides inside the hollow cap bore,
    # but we allow the overlap in case exact-mesh tessellation causes
    # marginal contact at the bore wall.
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="barrel_body",
        reason="Cap bore slides over the barrel with close radial clearance (friction-fit seating).",
    )
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="nib",
        reason="Seated cap encloses the nib inside its hollow bore (capture fit).",
    )
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="felt_tip",
        reason="Seated cap encloses the felt tip to keep the marker from drying out.",
    )

    return ctx.report()


object_model = build_object_model()
