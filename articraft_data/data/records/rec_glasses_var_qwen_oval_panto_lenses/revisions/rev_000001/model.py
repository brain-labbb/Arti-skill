from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Articraft brief
# ----------------------------------------------------------------------------
# Object: dark browline / clubmaster sunglasses, ~0.142 m total frame width.
# Coordinate frame:
#   +Y = wearer's left (model right), -Y = wearer's right
#   +Z = up
#   +X = backwards (toward the ears); the front lens plane sits near X = 0,
#        temple arms extend toward +X.
# Root/support: `frame_front` carries the chunky brow bar, bridge, nose pads,
#   full outline-matched metal rims, and the two metal hinge blocks. Lenses are mounted
#   into the frame front; temple arms fold at the hinge blocks.
# Articulations: two REVOLUTE joints (left_hinge, right_hinge) folding each
#   temple arm flat against the front of the frame. Both arms truly rotate.
# Materials: dark glossy gunmetal for brow + temples, smoky translucent for the
#   lenses, polished metal for the lower rim wire and hinge blocks.
# ----------------------------------------------------------------------------

# --- Key dimensions (meters) ---
LENS_W = 0.052          # single lens half-width span (full width)
LENS_H = 0.044          # lens height
LENS_GAP = 0.018        # bridge gap between the two lenses
LENS_CX = LENS_W / 2.0 + LENS_GAP / 2.0   # center offset of each lens in Y
LENS_THK = 0.0035       # lens thickness (along X)
FRONT_X = 0.0           # front face of lenses at X=0

BROW_DEPTH = 0.008      # brow bar thickness along X (refined, not blocky)
BROW_TOP_H = 0.009      # brow bar vertical band height (slim band)
BROW_TOP_H_BRIDGE = 0.0035  # band height over the nose bridge (thin)
HINGE_Y = LENS_CX + LENS_W / 2.0 - 0.004   # outer hinge Y position
HINGE_Z = LENS_H / 2.0 - 0.006             # hinge vertical position near top
TEMPLE_LEN = 0.135      # temple arm length along X


# ----------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ----------------------------------------------------------------------------

def _lens_outline_points(w: float, h: float, segments: int = 48) -> list[tuple[float, float]]:
    """Soft oval panto outline in local (Y, Z) coordinates."""
    hw, hh = w / 2.0, h / 2.0
    pts = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        sin_a = math.sin(a)
        cos_a = math.cos(a)
        ry = hw
        rz = hh
        if sin_a < 0.0:
            t = abs(sin_a)
            ry *= 1.0 + 0.06 * t
            rz *= 1.0 + 0.08 * t
        else:
            t = sin_a
            ry *= 1.0 - 0.04 * t
            rz *= 1.0 - 0.06 * t
        pts.append((ry * cos_a, rz * sin_a))
    return pts


def _lens_profile_wire(wp: cq.Workplane, w: float, h: float) -> cq.Workplane:
    """Soft oval panto lens outline: rounded top and fuller lower curve,
    drawn on the given workplane (YZ-style plane mapped to local XY)."""
    # Panto shape: smooth oval with slightly flatter/softer top and a fuller,
    # rounder bottom half. The full rim reuses this same point set.
    pts = _lens_outline_points(w, h)
    return wp.polyline(pts + [pts[0]]).close()


def build_lens(mirror: bool) -> cq.Workplane:
    """A smoky lens, slightly convex. Built in the YZ plane (depth along X)."""
    sign = 1.0 if mirror else -1.0
    cy = sign * LENS_CX
    # Work on a YZ plane positioned at the lens center.
    wp = cq.Workplane("YZ").workplane(offset=FRONT_X + LENS_THK / 2.0)
    prof = _lens_profile_wire(wp, LENS_W, LENS_H)
    solid = prof.extrude(LENS_THK)
    # Slight outward bow: fillet the front edges for a lens-like read.
    try:
        solid = solid.edges("|X").fillet(0.0016)
    except Exception:
        pass
    return solid.translate((0.0, cy, 0.0))


def build_lens_rim(mirror: bool) -> cq.Workplane:
    """Thin polished-metal full rim that follows the exact panto lens outline."""
    sign = 1.0 if mirror else -1.0
    cy = sign * LENS_CX
    x_rim = FRONT_X + LENS_THK / 2.0
    outline = _lens_outline_points(LENS_W, LENS_H, segments=64)
    spline_pts = [(x_rim, cy + y, z) for y, z in outline]
    spline_pts.append(spline_pts[0])
    return _sweep_tube_along(spline_pts, 0.0011)


def _sweep_tube_along(pts, radius: float) -> cq.Workplane:
    """Sweep a circular profile along a 3D spline (world coords)."""
    vecs = [cq.Vector(*p) for p in pts]
    # Build the path as a spline edge -> wire on a Workplane.
    spline_edge = cq.Edge.makeSpline(vecs)
    path = cq.Workplane(obj=cq.Wire.assembleEdges([spline_edge]))
    p0 = vecs[0]
    d = (vecs[1] - vecs[0]).normalized()
    prof = cq.Workplane(
        cq.Plane(origin=(p0.x, p0.y, p0.z), normal=(d.x, d.y, d.z))
    ).circle(radius)
    return prof.sweep(path, isFrenet=True)


def build_frame_front() -> cq.Workplane:
    """Chunky glossy brow bar + bridge + hinge blocks (dark gunmetal part)."""
    hw_total = LENS_CX + LENS_W / 2.0
    # --- Brow bar: a single continuous band across the top of both lenses. ---
    # Profile in the YZ plane that arcs slightly downward at the outer ends,
    # giving the browline silhouette; extruded along X for chunky depth.
    z_bot = LENS_H / 2.0 - 0.006
    # Build the brow outline as a closed loop in YZ.
    top_pts = []
    n = 40
    span = hw_total + 0.006
    for i in range(n + 1):
        t = i / n
        y = -span + 2.0 * span * t
        frac = abs(y) / span        # 0 at bridge, 1 at outer corners
        # Slim band whose height grows toward the temple ends and thins over
        # the bridge: interpolate band height from bridge -> outer with a
        # smooth (eased) curve so the top silhouette reads soft and tapered.
        ease = frac ** 1.4
        band_h = BROW_TOP_H_BRIDGE + (BROW_TOP_H - BROW_TOP_H_BRIDGE) * ease
        z = z_bot + band_h
        top_pts.append((y, z))
    bot_pts = []
    for i in range(n + 1):
        t = i / n
        y = span - 2.0 * span * t
        # bottom edge follows the top of the lenses with a small bridge dip
        z = z_bot - 0.004 * (abs(y) / span) ** 2
        bot_pts.append((y, z))
    loop = top_pts + bot_pts
    brow = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X - BROW_DEPTH)
        .polyline(loop)
        .close()
        .extrude(BROW_DEPTH + 0.002)
    )
    try:
        brow = brow.edges("|X").fillet(0.0015)
    except Exception:
        pass

    # --- Bridge: short connector over the nose between the brow halves. ---
    # It must overlap the brow's lower edge so the union stays one solid.
    bridge_h = LENS_H * 0.34
    bridge_cz = z_bot - bridge_h * 0.30  # top of bridge reaches up into brow
    bridge = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X - BROW_DEPTH)
        .center(0.0, bridge_cz)
        .rect(LENS_GAP + 0.010, bridge_h)
        .extrude(BROW_DEPTH + 0.002)
    )
    try:
        bridge = bridge.edges("|X").fillet(0.002)
    except Exception:
        pass
    front = brow.union(bridge)
    return front


def build_hinge_block(mirror: bool) -> cq.Workplane:
    """Polished metal hinge block at an outer top corner of the brow."""
    sign = 1.0 if mirror else -1.0
    cy = sign * HINGE_Y
    block = (
        cq.Workplane("XY")
        .box(0.010, 0.010, 0.012)
        .edges("|Z").fillet(0.0015)
    )
    return block.translate((FRONT_X, cy, HINGE_Z))


def build_nose_pad(mirror: bool) -> cq.Workplane:
    """Small nose pad on a thin stem, near the bridge."""
    sign = 1.0 if mirror else -1.0
    cy = sign * (LENS_GAP / 2.0 + 0.001)
    stem = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X)
        .center(cy, -LENS_H * 0.10)
        .circle(0.0009)
        .extrude(0.006)
    )
    pad = (
        cq.Workplane("XY")
        .ellipse(0.0035, 0.0065)
        .extrude(0.0018)
        .translate((FRONT_X + 0.006, cy, -LENS_H * 0.12))
    )
    return stem.union(pad)


def build_temple_arm() -> cq.Workplane:
    """Broad flat temple arm, wide near the hinge, tapering to the ear tip.

    Authored in a LOCAL frame whose origin is the hinge pivot, so it can be
    attached at the articulation frame directly. The arm extends along +X.
    """
    # Top-view (XY) tapered plate, then given thickness in Z.
    half_w0 = 0.012   # half height near hinge
    half_w1 = 0.004   # half height at ear
    L = TEMPLE_LEN
    thick = 0.006
    pts = [
        (0.0, -half_w0),
        (L * 0.55, -half_w1 * 1.6),
        (L * 0.92, -half_w1),
        (L, -half_w1 * 0.7),
        (L, half_w1 * 0.7),
        (L * 0.92, half_w1),
        (L * 0.55, half_w1 * 1.6),
        (0.0, half_w0),
    ]
    arm = (
        cq.Workplane("XY")
        .polyline(pts + [pts[0]])
        .close()
        .extrude(thick)
    )
    arm = arm.translate((0.0, 0.0, -thick / 2.0))
    try:
        arm = arm.edges("|Z").fillet(0.0012)
    except Exception:
        pass
    # Curved-down ear hook tip: small downward bend at the far end.
    hook = (
        cq.Workplane("XY")
        .center(L - 0.004, 0.0)
        .rect(0.012, 0.006)
        .extrude(0.018)
        .translate((0.0, 0.0, -0.018))
    )
    try:
        hook = hook.edges("|Y").fillet(0.0015)
    except Exception:
        pass
    arm = arm.union(hook)
    # Small hinge knuckle at the pivot end so it reads as a real hinge mate.
    knuckle = (
        cq.Workplane("XY")
        .box(0.008, 0.012, 0.012)
        .edges("|Z").fillet(0.0012)
        .translate((0.001, 0.0, 0.0))
    )
    return arm.union(knuckle)


# ----------------------------------------------------------------------------
# Model assembly
# ----------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panto_browline_sunglasses")

    glossy_dark = model.material(
        "glossy_gunmetal", rgba=(0.16, 0.17, 0.19, 1.0)
    )
    polished_metal = model.material(
        "polished_metal", rgba=(0.62, 0.64, 0.66, 1.0)
    )
    smoky_lens = model.material(
        "smoky_lens", rgba=(0.10, 0.11, 0.13, 0.55)
    )

    # --- Front frame (root) ---
    frame = model.part("frame_front")
    frame.visual(
        mesh_from_cadquery(build_frame_front(), "brow_bar"),
        material=glossy_dark,
        name="brow_bar",
    )
    frame.visual(
        mesh_from_cadquery(build_hinge_block(False), "hinge_block_right"),
        material=polished_metal,
        name="hinge_block_right",
    )
    frame.visual(
        mesh_from_cadquery(build_hinge_block(True), "hinge_block_left"),
        material=polished_metal,
        name="hinge_block_left",
    )
    frame.visual(
        mesh_from_cadquery(build_lens_rim(False), "lens_rim_right"),
        material=polished_metal,
        name="lens_rim_right",
    )
    frame.visual(
        mesh_from_cadquery(build_lens_rim(True), "lens_rim_left"),
        material=polished_metal,
        name="lens_rim_left",
    )
    frame.visual(
        mesh_from_cadquery(build_nose_pad(False), "nose_pad_right"),
        material=polished_metal,
        name="nose_pad_right",
    )
    frame.visual(
        mesh_from_cadquery(build_nose_pad(True), "nose_pad_left"),
        material=polished_metal,
        name="nose_pad_left",
    )

    # --- Lenses (rigidly fixed to the frame front) ---
    lens_r = model.part("lens_right")
    lens_r.visual(
        mesh_from_cadquery(build_lens(False), "lens_right"),
        material=smoky_lens,
        name="lens_right",
    )
    lens_l = model.part("lens_left")
    lens_l.visual(
        mesh_from_cadquery(build_lens(True), "lens_left"),
        material=smoky_lens,
        name="lens_left",
    )
    model.articulation(
        "frame_to_lens_right",
        ArticulationType.FIXED,
        parent=frame,
        child=lens_r,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_lens_left",
        ArticulationType.FIXED,
        parent=frame,
        child=lens_l,
        origin=Origin(),
    )

    # --- Temple arms (folding revolute joints at hinge blocks) ---
    arm_r = model.part("temple_right")
    arm_r.visual(
        mesh_from_cadquery(build_temple_arm(), "temple_right"),
        material=glossy_dark,
        name="temple_right",
    )
    arm_l = model.part("temple_left")
    arm_l.visual(
        mesh_from_cadquery(build_temple_arm(), "temple_left"),
        material=glossy_dark,
        name="temple_left",
    )

    # Right hinge (-Y side). At q=0 the arm extends straight back (+X, open).
    # Folding the arm inward (toward -Y, across the front) is positive about +Z.
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=arm_r,
        origin=Origin(xyz=(FRONT_X + 0.005, -HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(95.0)
        ),
    )
    # Left hinge (+Y side). Mirror: negate axis so positive folds inward.
    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=arm_l,
        origin=Origin(xyz=(FRONT_X + 0.005, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(95.0)
        ),
    )

    return model


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame_front")
    lens_r = object_model.get_part("lens_right")
    lens_l = object_model.get_part("lens_left")
    arm_r = object_model.get_part("temple_right")
    arm_l = object_model.get_part("temple_left")

    right_hinge = object_model.get_articulation("right_hinge")
    left_hinge = object_model.get_articulation("left_hinge")

    # The temple-arm hinge knuckle is intentionally captured inside the metal
    # hinge block proxy so the fold mate reads as a real hinge.
    ctx.allow_overlap(
        "frame_front",
        "temple_right",
        elem_a="hinge_block_right",
        elem_b="temple_right",
        reason="Temple hinge knuckle is captured inside the right hinge block.",
    )
    ctx.allow_overlap(
        "frame_front",
        "temple_left",
        elem_a="hinge_block_left",
        elem_b="temple_left",
        reason="Temple hinge knuckle is captured inside the left hinge block.",
    )

    # (1) Exactly two temple-arm revolute joints that fold meaningfully.
    revolute_temple_joints = [
        j
        for j in object_model.articulations
        if j.articulation_type == ArticulationType.REVOLUTE
        and j.child in ("temple_right", "temple_left")
    ]
    ctx.check(
        "exactly two temple revolute joints",
        len(revolute_temple_joints) == 2,
        details=f"found {[j.name for j in revolute_temple_joints]}",
    )
    for j in revolute_temple_joints:
        lim = j.motion_limits
        sweep = abs((lim.upper or 0.0) - (lim.lower or 0.0))
        ctx.check(
            f"{j.name} folds through a meaningful angle",
            sweep >= math.radians(60.0),
            details=f"sweep={math.degrees(sweep):.1f} deg",
        )

    # Prove both arms actually rotate: their far-tip world position must change
    # significantly between open and folded poses.
    open_r = ctx.part_world_aabb(arm_r)
    open_l = ctx.part_world_aabb(arm_l)
    with ctx.pose({right_hinge: math.radians(90.0), left_hinge: math.radians(90.0)}):
        fold_r = ctx.part_world_aabb(arm_r)
        fold_l = ctx.part_world_aabb(arm_l)

    def _max_extent(aabb):
        (lo, hi) = aabb
        return [hi[i] - lo[i] for i in range(3)]

    # When open, the arm is long in X; when folded, long in Y.
    ext_open_r = _max_extent(open_r)
    ext_fold_r = _max_extent(fold_r)
    ctx.check(
        "right temple swings from along-X to across-Y when folded",
        ext_open_r[0] > 0.10 and ext_fold_r[1] > ext_open_r[1] + 0.05,
        details=f"open={ext_open_r}, folded={ext_fold_r}",
    )
    ext_open_l = _max_extent(open_l)
    ext_fold_l = _max_extent(fold_l)
    ctx.check(
        "left temple swings from along-X to across-Y when folded",
        ext_open_l[0] > 0.10 and ext_fold_l[1] > ext_open_l[1] + 0.05,
        details=f"open={ext_open_l}, folded={ext_fold_l}",
    )

    # (2) A single continuous brow bar spanning across both eyes.
    brow = frame.get_visual("brow_bar")
    ctx.check("brow bar visual exists", brow is not None)
    ctx.expect_overlap(
        lens_r, frame, axes="y", elem_b="brow_bar", min_overlap=0.01,
        name="brow spans over right lens",
    )
    ctx.expect_overlap(
        lens_l, frame, axes="y", elem_b="brow_bar", min_overlap=0.01,
        name="brow spans over left lens",
    )
    # The refined brow must read as a SLIM band, not a heavy slab: its rise
    # above the lens top should occupy only a small fraction of the lens height.
    brow_aabb = ctx.part_element_world_aabb(frame, elem="brow_bar")
    if brow_aabb is not None:
        rise_above_lens = brow_aabb[1][2] - (LENS_H / 2.0)
        ctx.check(
            "brow band is slim above the lens top",
            rise_above_lens <= 0.012,
            details=f"rise={rise_above_lens:.4f} m (<= ~25% of lens height)",
        )

    # (3) Two smoky lenses, each wrapped by a matching full metal rim.
    ctx.check("right lens exists", lens_r.get_visual("lens_right") is not None)
    ctx.check("left lens exists", lens_l.get_visual("lens_left") is not None)
    ctx.check(
        "right full lens rim exists",
        frame.get_visual("lens_rim_right") is not None,
    )
    ctx.check(
        "left full lens rim exists",
        frame.get_visual("lens_rim_left") is not None,
    )
    # Each rim follows the lens outline and overlaps its lens footprint.
    ctx.expect_overlap(
        lens_r, frame, axes="y", elem_b="lens_rim_right", min_overlap=0.01,
        name="right rim wraps right lens",
    )
    ctx.expect_overlap(
        lens_l, frame, axes="y", elem_b="lens_rim_left", min_overlap=0.01,
        name="left rim wraps left lens",
    )

    # (4) Brow bar uses a dark glossy material.
    mat = brow.material
    mat_name = mat.name if hasattr(mat, "name") else mat
    ctx.check(
        "brow bar uses dark glossy material",
        mat_name == "glossy_gunmetal",
        details=f"material={mat_name}",
    )
    # Confirm the dark glossy material is genuinely dark.
    dark_mat = next(m for m in object_model.materials if m.name == "glossy_gunmetal")
    ctx.check(
        "glossy material is dark",
        max(dark_mat.rgba[:3]) < 0.35,
        details=f"rgba={dark_mat.rgba}",
    )

    # (5) No floating/disconnected parts handled by baseline QC. Confirm lenses
    # and temples physically connect to the frame.
    ctx.expect_contact(arm_r, frame, name="right temple connects to frame")
    ctx.expect_contact(arm_l, frame, name="left temple connects to frame")
    ctx.expect_contact(lens_r, frame, name="right lens connects to frame")
    ctx.expect_contact(lens_l, frame, name="left lens connects to frame")

    return ctx.report()


object_model = build_object_model()
