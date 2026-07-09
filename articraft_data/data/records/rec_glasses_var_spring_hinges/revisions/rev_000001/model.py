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
# Object: dark browline / clubmaster sunglasses with SPRING-HINGE barrels,
#   ~0.142 m total frame width. Variant fork: hinge blocks replaced by larger
#   cylindrical spring-hinge barrels + secondary flex links at temple roots.
# Coordinate frame:
#   +Y = wearer's left (model right), -Y = wearer's right
#   +Z = up
#   +X = backwards (toward the ears); the front lens plane sits near X = 0,
#        temple arms extend toward +X.
# Root/support: `frame_front` carries the chunky brow bar, bridge, nose pads,
#   full outline-matched metal rims, and the two spring-hinge barrels.
# Articulations:
#   - frame → flex_link (REVOLUTE, spring action, small range ~0-20°)
#   - flex_link → temple_arm (REVOLUTE, main fold, 0-95°)
#   Both sides mirrored. 4 revolute joints total.
# Materials: dark glossy gunmetal for brow + temples, smoky translucent for the
#   lenses, polished metal for the full rim wire and spring barrels.
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

# Spring-hinge barrel dimensions (larger than old box hinge blocks)
BARREL_R = 0.005        # barrel radius
BARREL_LEN = 0.014      # barrel length along Y axis (hinge pin direction)

# Flex link dimensions
FLEX_LEN = 0.012        # flex link length along X
FLEX_W = 0.010          # flex link width along Y
FLEX_THK = 0.005        # flex link thickness along Z

# Spring flex range (radians)
SPRING_LOWER = 0.0
SPRING_UPPER = math.radians(20.0)


# ----------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ----------------------------------------------------------------------------

def _lens_outline_points(w: float, h: float) -> list[tuple[float, float]]:
    """Rounded-rectangular lens/rim outline in local (Y, Z) coordinates."""
    hw, hh = w / 2.0, h / 2.0
    return [
        (-hw, hh * 0.55),
        (-hw * 0.55, hh),
        (hw * 0.7, hh),
        (hw, hh * 0.35),
        (hw * 0.92, -hh * 0.55),
        (hw * 0.5, -hh),
        (-hw * 0.45, -hh),
        (-hw, -hh * 0.25),
    ]


def _lens_profile_wire(wp: cq.Workplane, w: float, h: float) -> cq.Workplane:
    """Rounded-rectangular lens outline, slightly tapered toward outer corner,
    drawn on the given workplane (YZ-style plane mapped to local XY)."""
    pts = _lens_outline_points(w, h)
    return wp.polyline(pts + [pts[0]]).close()


def build_lens(mirror: bool) -> cq.Workplane:
    """A smoky lens, slightly convex. Built in the YZ plane (depth along X)."""
    sign = 1.0 if mirror else -1.0
    cy = sign * LENS_CX
    wp = cq.Workplane("YZ").workplane(offset=FRONT_X + LENS_THK / 2.0)
    prof = _lens_profile_wire(wp, LENS_W, LENS_H)
    solid = prof.extrude(LENS_THK)
    try:
        solid = solid.edges("|X").fillet(0.0016)
    except Exception:
        pass
    return solid.translate((0.0, cy, 0.0))


def build_lens_rim(mirror: bool) -> cq.Workplane:
    """Thin polished-metal full rim that follows the exact lens outline."""
    sign = 1.0 if mirror else -1.0
    cy = sign * LENS_CX
    x_rim = FRONT_X + LENS_THK / 2.0
    rim_pts = [(x_rim, cy + y, z) for y, z in _lens_outline_points(LENS_W, LENS_H)]
    return _sweep_tube_along(rim_pts + [rim_pts[0]], 0.0011, smooth=False)


def _sweep_tube_along(pts, radius: float, smooth: bool = True) -> cq.Workplane:
    """Sweep a circular profile along a 3D path in world coordinates."""
    vecs = [cq.Vector(*p) for p in pts]
    if smooth:
        edges = [cq.Edge.makeSpline(vecs)]
    else:
        edges = [cq.Edge.makeLine(vecs[i], vecs[i + 1]) for i in range(len(vecs) - 1)]
    path = cq.Workplane(obj=cq.Wire.assembleEdges(edges))
    p0 = vecs[0]
    d = (vecs[1] - vecs[0]).normalized()
    prof = cq.Workplane(
        cq.Plane(origin=(p0.x, p0.y, p0.z), normal=(d.x, d.y, d.z))
    ).circle(radius)
    return prof.sweep(path, isFrenet=True)


def build_frame_front() -> cq.Workplane:
    """Chunky glossy brow bar + bridge (dark gunmetal part)."""
    hw_total = LENS_CX + LENS_W / 2.0
    z_bot = LENS_H / 2.0 - 0.006
    top_pts = []
    n = 40
    span = hw_total + 0.006
    for i in range(n + 1):
        t = i / n
        y = -span + 2.0 * span * t
        frac = abs(y) / span
        ease = frac ** 1.4
        band_h = BROW_TOP_H_BRIDGE + (BROW_TOP_H - BROW_TOP_H_BRIDGE) * ease
        z = z_bot + band_h
        top_pts.append((y, z))
    bot_pts = []
    for i in range(n + 1):
        t = i / n
        y = span - 2.0 * span * t
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

    bridge_h = LENS_H * 0.34
    bridge_cz = z_bot - bridge_h * 0.30
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


def build_spring_barrel(mirror: bool) -> cq.Workplane:
    """Larger cylindrical spring-hinge barrel at an outer top corner.

    The barrel is a cylinder aligned along the Y axis (hinge pin direction),
    with end-cap rings and a mounting flange that connects to the brow bar.
    Everything is built at Z=0, then translated to HINGE_Z at the end.
    """
    sign = 1.0 if mirror else -1.0
    cy = sign * HINGE_Y

    # Main barrel cylinder aligned along Y, centered at Z=0.
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=cy - sign * BARREL_LEN / 2.0)
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
    )

    # Mounting flange: a tab bridging from the barrel center to the brow bar.
    # The brow bar rear face is near X = FRONT_X + 0.002; the barrel center is
    # at X = 0. The flange spans that gap so the barrel reads as mounted.
    flange_x_center = (FRONT_X + 0.002 + 0.0) / 2.0
    flange_dx = abs(FRONT_X + 0.002 - 0.0) + 0.002
    flange = (
        cq.Workplane("XY")
        .box(flange_dx, 0.006, BARREL_R * 2.4)
        .translate((flange_x_center, cy, 0.0))
    )

    # End-cap rings that overlap the barrel ends for a visible barrel detail.
    cap_inner = (
        cq.Workplane("XZ")
        .workplane(offset=cy - sign * (BARREL_LEN / 2.0 - 0.001))
        .circle(BARREL_R + 0.0008)
        .extrude(0.002)
    )
    cap_outer = (
        cq.Workplane("XZ")
        .workplane(offset=cy + sign * (BARREL_LEN / 2.0 - 0.003))
        .circle(BARREL_R + 0.0008)
        .extrude(0.002)
    )

    result = barrel.union(flange).union(cap_inner).union(cap_outer)
    return result.translate((0.0, 0.0, HINGE_Z))


def build_flex_link() -> cq.Workplane:
    """Small secondary flex link at the temple root.

    A flat, rounded-rectangular plate that sits between the spring barrel and
    the temple arm. Authored in LOCAL frame with origin at the pivot point
    (where it connects to the barrel), extending along +X toward the temple.
    """
    # Rounded rectangular plate.
    pts = [
        (0.0, -FLEX_W / 2.0),
        (FLEX_LEN * 0.9, -FLEX_W / 2.0),
        (FLEX_LEN, -FLEX_W * 0.35),
        (FLEX_LEN, FLEX_W * 0.35),
        (FLEX_LEN * 0.9, FLEX_W / 2.0),
        (0.0, FLEX_W / 2.0),
    ]
    link = (
        cq.Workplane("XY")
        .polyline(pts + [pts[0]])
        .close()
        .extrude(FLEX_THK)
    )
    link = link.translate((0.0, 0.0, -FLEX_THK / 2.0))
    try:
        link = link.edges("|Z").fillet(0.001)
    except Exception:
        pass
    # Small cylindrical boss at the pivot end (barrel mating socket).
    boss = (
        cq.Workplane("XY")
        .circle(BARREL_R * 0.75)
        .extrude(FLEX_THK * 0.8)
        .translate((0.0, 0.0, -FLEX_THK * 0.4))
    )
    # Small cylindrical boss at the temple end (temple mating pin).
    pin = (
        cq.Workplane("XY")
        .circle(0.002)
        .extrude(FLEX_THK * 0.6)
        .translate((FLEX_LEN, 0.0, -FLEX_THK * 0.3))
    )
    return link.union(boss).union(pin)


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

    Authored in a LOCAL frame whose origin is the temple-side end of the
    flex link (so it attaches at the articulation frame directly).
    The arm extends along +X.
    """
    half_w0 = 0.012
    half_w1 = 0.004
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
    # Curved-down ear hook tip.
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
    # Small hinge knuckle at the pivot end for the flex link mate.
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
    model = ArticulatedObject(name="clubmaster_sunglasses_spring_hinge")

    glossy_dark = model.material(
        "glossy_gunmetal", rgba=(0.16, 0.17, 0.19, 1.0)
    )
    polished_metal = model.material(
        "polished_metal", rgba=(0.62, 0.64, 0.66, 1.0)
    )
    smoky_lens = model.material(
        "smoky_lens", rgba=(0.10, 0.11, 0.13, 0.55)
    )
    flex_dark = model.material(
        "flex_gunmetal", rgba=(0.20, 0.21, 0.23, 1.0)
    )

    # --- Front frame (root) ---
    frame = model.part("frame_front")
    frame.visual(
        mesh_from_cadquery(build_frame_front(), "brow_bar"),
        material=glossy_dark,
        name="brow_bar",
    )

    # Spring-hinge barrels (larger cylindrical barrels replacing old hinge blocks)
    for i, mirror in enumerate([False, True]):
        side = "right" if not mirror else "left"
        frame.visual(
            mesh_from_cadquery(build_spring_barrel(mirror), f"spring_barrel_{side}"),
            material=polished_metal,
            name=f"spring_barrel_{side}",
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

    # --- Flex links (secondary articulation at temple root) ---
    # Each flex link is a small intermediate part between the spring barrel
    # and the temple arm. The spring joint allows a small angular range.
    flex_r = model.part("flex_link_right")
    flex_r.visual(
        mesh_from_cadquery(build_flex_link(), "flex_link_right"),
        material=flex_dark,
        name="flex_link_right",
    )
    flex_l = model.part("flex_link_left")
    flex_l.visual(
        mesh_from_cadquery(build_flex_link(), "flex_link_left"),
        material=flex_dark,
        name="flex_link_left",
    )

    # Right spring flex joint (-Y side). Frame → flex_link.
    # Origin at the barrel center. At q=0, the flex link extends along +X.
    # Axis +Z so positive rotation flexes the link outward (away from head).
    model.articulation(
        "right_spring",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flex_r,
        origin=Origin(xyz=(FRONT_X + 0.005, -HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=SPRING_LOWER, upper=SPRING_UPPER,
        ),
    )
    # Left spring flex joint (+Y side). Mirror axis.
    model.articulation(
        "left_spring",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flex_l,
        origin=Origin(xyz=(FRONT_X + 0.005, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=SPRING_LOWER, upper=SPRING_UPPER,
        ),
    )

    # --- Temple arms (main folding revolute joints) ---
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

    # Right hinge: flex_link → temple_arm. At q=0 the arm extends straight
    # back (+X, open). The joint origin is at the far end of the flex link.
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=flex_r,
        child=arm_r,
        origin=Origin(xyz=(FLEX_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(95.0),
        ),
    )
    # Left hinge: mirror axis so positive folds inward.
    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=flex_l,
        child=arm_l,
        origin=Origin(xyz=(FLEX_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(95.0),
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
    flex_r = object_model.get_part("flex_link_right")
    flex_l = object_model.get_part("flex_link_left")
    arm_r = object_model.get_part("temple_right")
    arm_l = object_model.get_part("temple_left")

    right_hinge = object_model.get_articulation("right_hinge")
    left_hinge = object_model.get_articulation("left_hinge")
    right_spring = object_model.get_articulation("right_spring")
    left_spring = object_model.get_articulation("left_spring")

    # --- Intentional overlaps ---
    # The flex link boss is intentionally captured inside the spring barrel.
    ctx.allow_overlap(
        "frame_front",
        "flex_link_right",
        elem_a="spring_barrel_right",
        elem_b="flex_link_right",
        reason="Flex link boss is captured inside the right spring barrel.",
    )
    ctx.allow_overlap(
        "frame_front",
        "flex_link_left",
        elem_a="spring_barrel_left",
        elem_b="flex_link_left",
        reason="Flex link boss is captured inside the left spring barrel.",
    )
    # Temple knuckle captured inside flex link at the temple-side pin.
    ctx.allow_overlap(
        "flex_link_right",
        "temple_right",
        elem_a="flex_link_right",
        elem_b="temple_right",
        reason="Temple hinge knuckle is captured inside the right flex link.",
    )
    ctx.allow_overlap(
        "flex_link_left",
        "temple_left",
        elem_a="flex_link_left",
        elem_b="temple_left",
        reason="Temple hinge knuckle is captured inside the left flex link.",
    )

    # --- (1) Exactly two temple-arm revolute joints that fold meaningfully ---
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

    # --- (1b) Spring-hinge flex joints exist with small angular range ---
    spring_joints = [
        j
        for j in object_model.articulations
        if j.articulation_type == ArticulationType.REVOLUTE
        and j.child in ("flex_link_right", "flex_link_left")
    ]
    ctx.check(
        "exactly two spring flex revolute joints",
        len(spring_joints) == 2,
        details=f"found {[j.name for j in spring_joints]}",
    )
    for j in spring_joints:
        lim = j.motion_limits
        sweep = abs((lim.upper or 0.0) - (lim.lower or 0.0))
        ctx.check(
            f"{j.name} has a small spring range (5-30 deg)",
            math.radians(5.0) <= sweep <= math.radians(30.0),
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

    # --- (2) A single continuous brow bar spanning across both eyes ---
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
    brow_aabb = ctx.part_element_world_aabb(frame, elem="brow_bar")
    if brow_aabb is not None:
        rise_above_lens = brow_aabb[1][2] - (LENS_H / 2.0)
        ctx.check(
            "brow band is slim above the lens top",
            rise_above_lens <= 0.012,
            details=f"rise={rise_above_lens:.4f} m (<= ~25% of lens height)",
        )

    # --- (3) Two smoky lenses, each wrapped by a matching full metal rim ---
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
    ctx.expect_overlap(
        lens_r, frame, axes="y", elem_b="lens_rim_right", min_overlap=0.01,
        name="right rim wraps right lens",
    )
    ctx.expect_overlap(
        lens_l, frame, axes="y", elem_b="lens_rim_left", min_overlap=0.01,
        name="left rim wraps left lens",
    )

    # --- (3b) Spring-hinge barrels exist and are visible ---
    ctx.check(
        "right spring barrel exists",
        frame.get_visual("spring_barrel_right") is not None,
    )
    ctx.check(
        "left spring barrel exists",
        frame.get_visual("spring_barrel_left") is not None,
    )
    # Barrels should be larger than a simple small block - check their extent.
    barrel_r_aabb = ctx.part_element_world_aabb(frame, elem="spring_barrel_right")
    if barrel_r_aabb is not None:
        barrel_dy = barrel_r_aabb[1][1] - barrel_r_aabb[0][1]
        ctx.check(
            "right spring barrel has meaningful length along Y",
            barrel_dy >= 0.010,
            details=f"barrel_dy={barrel_dy:.4f} m",
        )

    # --- (4) Brow bar uses a dark glossy material ---
    mat = brow.material
    mat_name = mat.name if hasattr(mat, "name") else mat
    ctx.check(
        "brow bar uses dark glossy material",
        mat_name == "glossy_gunmetal",
        details=f"material={mat_name}",
    )
    dark_mat = next(m for m in object_model.materials if m.name == "glossy_gunmetal")
    ctx.check(
        "glossy material is dark",
        max(dark_mat.rgba[:3]) < 0.35,
        details=f"rgba={dark_mat.rgba}",
    )

    # --- (5) No floating/disconnected parts ---
    # Flex links connect to frame via spring barrels.
    ctx.expect_contact(flex_r, frame, name="right flex link connects to frame")
    ctx.expect_contact(flex_l, frame, name="left flex link connects to frame")
    # Temple arms connect to flex links.
    ctx.expect_contact(arm_r, flex_r, name="right temple connects to flex link")
    ctx.expect_contact(arm_l, flex_l, name="left temple connects to flex link")
    # Lenses connect to frame.
    ctx.expect_contact(lens_r, frame, name="right lens connects to frame")
    ctx.expect_contact(lens_l, frame, name="left lens connects to frame")

    # --- Flex link geometry check: small intermediate part ---
    flex_r_aabb = ctx.part_world_aabb(flex_r)
    if flex_r_aabb is not None:
        flex_dx = flex_r_aabb[1][0] - flex_r_aabb[0][0]
        ctx.check(
            "right flex link is a small part (length < 0.020 m)",
            flex_dx < 0.020,
            details=f"flex_dx={flex_dx:.4f} m",
        )

    return ctx.report()


object_model = build_object_model()
