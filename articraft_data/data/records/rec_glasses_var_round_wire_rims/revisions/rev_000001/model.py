from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Articraft brief – fork: round wire-rim sunglasses
# ----------------------------------------------------------------------------
# Forked from browline/clubmaster parent. Changed ONLY the front rim shape:
#   - chunky brow bar + thin lower half-rims → thin round wire rims (torus)
#   - rounded-rectangular lenses → circular lenses
# Kept identical: bridge concept, nose pads, hinge blocks, temple arms, two
#   revolute temple-fold joints.
#
# Coordinate frame (same as parent):
#   +Y = wearer's left, -Y = wearer's right
#   +Z = up
#   +X = backward (toward ears); front lens plane near X = 0
#
# Root: frame_front carries two circular wire rims, bridge, hinge blocks,
#   nose pads. Lenses are FIXED to the frame. Temple arms fold at hinges.
# ----------------------------------------------------------------------------

# --- Key dimensions (meters) ---
LENS_RADIUS = 0.024         # circular lens radius (48 mm diameter)
LENS_GAP = 0.018            # bridge gap between the two lenses
LENS_CX = LENS_RADIUS + LENS_GAP / 2.0   # center Y-offset of each lens
LENS_THK = 0.003            # lens thickness (along X)
FRONT_X = 0.0               # front face of lenses at X = 0

RIM_WIRE_RADIUS = 0.0012    # thin wire rim tube radius
BRIDGE_Z = 0.005            # bridge bar height above lens center

HINGE_Y = LENS_CX + LENS_RADIUS          # outer hinge Y (at rim edge)
HINGE_Z = 0.0                            # hinge at lens center height
HINGE_X_OFFSET = 0.004                   # hinge pivot behind rim center
TEMPLE_LEN = 0.135                       # temple arm length along X


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------

def _sign(i: int) -> float:
    """Return -1 for i==0 (right / -Y side), +1 for i==1 (left / +Y side)."""
    return -1.0 if i == 0 else 1.0


def build_rim(i: int) -> TorusGeometry:
    """Thin round wire rim – full circle (torus) around a circular lens."""
    cy = _sign(i) * LENS_CX
    rim = TorusGeometry(
        LENS_RADIUS, RIM_WIRE_RADIUS,
        radial_segments=20, tubular_segments=48,
    )
    # Torus starts in XY plane (axis Z). Rotate 90° around Y → YZ plane (axis X).
    rim.rotate_y(math.pi / 2.0)
    rim.translate(FRONT_X + LENS_THK / 2.0, cy, 0.0)
    return rim


def build_lens(i: int) -> cq.Workplane:
    """Circular lens disk, slightly convex (filleted edges)."""
    cy = _sign(i) * LENS_CX
    lens = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X)
        .circle(LENS_RADIUS)
        .extrude(LENS_THK)
    )
    try:
        lens = lens.edges().fillet(LENS_THK * 0.25)
    except Exception:
        pass
    return lens.translate((0.0, cy, 0.0))


def build_bridge() -> BoxGeometry:
    """Short bridge bar connecting the upper-inner portions of the two rims."""
    dy = math.sqrt(max(0.0, LENS_RADIUS ** 2 - BRIDGE_Z ** 2))
    span = 2.0 * (LENS_CX - dy)       # gap between rims at BRIDGE_Z
    bridge_w = span + 0.006            # extend into rims for overlap
    bridge_h = 0.005
    bridge_d = 0.005
    bridge = BoxGeometry((bridge_d, bridge_w, bridge_h))
    bridge.translate(FRONT_X + LENS_THK / 2.0, 0.0, BRIDGE_Z)
    return bridge


def build_hinge_block(i: int) -> cq.Workplane:
    """Polished metal hinge block at the outer rim edge."""
    cy = _sign(i) * HINGE_Y
    block = (
        cq.Workplane("XY")
        .box(0.010, 0.010, 0.012)
        .edges("|Z").fillet(0.0015)
    )
    return block.translate((FRONT_X + LENS_THK / 2.0, cy, HINGE_Z))


def build_nose_pad(i: int) -> cq.Workplane:
    """Small nose pad on a thin stem, attached to the bridge."""
    sign = _sign(i)
    cy = sign * (LENS_GAP / 2.0 + 0.002)
    x_base = FRONT_X + LENS_THK / 2.0 + 0.002

    # Stem from bridge down to pad position
    stem_top_z = BRIDGE_Z + 0.002      # extend into bridge for overlap
    stem_bot_z = -LENS_RADIUS * 0.45   # below lens center
    stem_h = stem_top_z - stem_bot_z

    stem = (
        cq.Workplane("XY")
        .center(x_base, cy)
        .circle(0.0009)
        .extrude(stem_h)
        .translate((0.0, 0.0, stem_bot_z))
    )
    pad = (
        cq.Workplane("XY")
        .center(x_base, cy)
        .ellipse(0.0035, 0.0065)
        .extrude(0.002)
        .translate((0.0, 0.0, stem_bot_z - 0.002))
    )
    return stem.union(pad)


def build_temple_arm() -> cq.Workplane:
    """Broad flat temple arm, wide near the hinge, tapering to the ear tip.

    Authored in a LOCAL frame whose origin is the hinge pivot, so it can be
    attached at the articulation frame directly. The arm extends along +X.
    """
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
    # Curved-down ear hook tip
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
    # Hinge knuckle at the pivot end
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
    model = ArticulatedObject(name="round_wire_sunglasses")

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

    # Repeated frame sub-parts: rims, hinge blocks, nose pads
    for i in range(2):
        frame.visual(
            mesh_from_geometry(build_rim(i), f"rim_{i}"),
            material=polished_metal,
            name=f"rim_{i}",
        )
    frame.visual(
        mesh_from_geometry(build_bridge(), "bridge"),
        material=polished_metal,
        name="bridge",
    )
    for i in range(2):
        frame.visual(
            mesh_from_cadquery(build_hinge_block(i), f"hinge_block_{i}"),
            material=polished_metal,
            name=f"hinge_block_{i}",
        )
    for i in range(2):
        frame.visual(
            mesh_from_cadquery(build_nose_pad(i), f"nose_pad_{i}"),
            material=polished_metal,
            name=f"nose_pad_{i}",
        )

    # --- Lenses (rigidly fixed to the frame) ---
    lens_parts = []
    for i in range(2):
        lp = model.part(f"lens_{i}")
        lp.visual(
            mesh_from_cadquery(build_lens(i), f"lens_{i}"),
            material=smoky_lens,
            name=f"lens_{i}",
        )
        model.articulation(
            f"frame_to_lens_{i}",
            ArticulationType.FIXED,
            parent=frame,
            child=lp,
            origin=Origin(),
        )
        lens_parts.append(lp)

    # --- Temple arms (folding revolute joints at hinge blocks) ---
    temple_parts = []
    hinge_x = FRONT_X + LENS_THK / 2.0 + HINGE_X_OFFSET
    for i in range(2):
        tp = model.part(f"temple_{i}")
        tp.visual(
            mesh_from_cadquery(build_temple_arm(), f"temple_{i}"),
            material=glossy_dark,
            name=f"temple_{i}",
        )
        temple_parts.append(tp)

    # Right hinge (i=0, -Y side). Positive q about +Z folds arm inward.
    model.articulation(
        "hinge_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=temple_parts[0],
        origin=Origin(xyz=(hinge_x, -HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=math.radians(95.0),
        ),
    )
    # Left hinge (i=1, +Y side). Negate axis so positive folds inward.
    model.articulation(
        "hinge_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=temple_parts[1],
        origin=Origin(xyz=(hinge_x, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=math.radians(95.0),
        ),
    )

    return model


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame_front")
    lens_0 = object_model.get_part("lens_0")
    lens_1 = object_model.get_part("lens_1")
    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")

    hinge_0 = object_model.get_articulation("hinge_0")
    hinge_1 = object_model.get_articulation("hinge_1")

    # --- Overlap allowances ---
    # Temple hinge knuckle captured inside the hinge block (same as parent).
    for i in range(2):
        ctx.allow_overlap(
            "frame_front",
            f"temple_{i}",
            elem_a=f"hinge_block_{i}",
            elem_b=f"temple_{i}",
            reason=f"Temple hinge knuckle is captured inside hinge block {i}.",
        )
    # Small lens-edge / temple-knuckle intersection at the hinge mount.
    for i in range(2):
        ctx.allow_overlap(
            f"lens_{i}",
            f"temple_{i}",
            elem_a=f"lens_{i}",
            elem_b=f"temple_{i}",
            reason=f"Temple knuckle slightly intersects lens {i} edge at the hinge mount.",
        )

    # --- (1) Exactly two temple-arm revolute joints that fold meaningfully ---
    revolute_hinges = [
        j for j in object_model.articulations
        if j.articulation_type == ArticulationType.REVOLUTE
        and j.child in ("temple_0", "temple_1")
    ]
    ctx.check(
        "exactly two temple revolute joints",
        len(revolute_hinges) == 2,
        details=f"found {[j.name for j in revolute_hinges]}",
    )
    for j in revolute_hinges:
        lim = j.motion_limits
        sweep = abs((lim.upper or 0.0) - (lim.lower or 0.0))
        ctx.check(
            f"{j.name} folds through a meaningful angle",
            sweep >= math.radians(60.0),
            details=f"sweep={math.degrees(sweep):.1f} deg",
        )

    # Prove both arms actually rotate between open and folded poses.
    open_0 = ctx.part_world_aabb(temple_0)
    open_1 = ctx.part_world_aabb(temple_1)
    with ctx.pose({hinge_0: math.radians(90.0), hinge_1: math.radians(90.0)}):
        fold_0 = ctx.part_world_aabb(temple_0)
        fold_1 = ctx.part_world_aabb(temple_1)

    def _max_extent(aabb):
        (lo, hi) = aabb
        return [hi[k] - lo[k] for k in range(3)]

    ext_open_0 = _max_extent(open_0)
    ext_fold_0 = _max_extent(fold_0)
    ctx.check(
        "temple_0 swings from along-X to across-Y when folded",
        ext_open_0[0] > 0.10 and ext_fold_0[1] > ext_open_0[1] + 0.05,
        details=f"open={ext_open_0}, folded={ext_fold_0}",
    )
    ext_open_1 = _max_extent(open_1)
    ext_fold_1 = _max_extent(fold_1)
    ctx.check(
        "temple_1 swings from along-X to across-Y when folded",
        ext_open_1[0] > 0.10 and ext_fold_1[1] > ext_open_1[1] + 0.05,
        details=f"open={ext_open_1}, folded={ext_fold_1}",
    )

    # --- (2) Two thin round wire rims (not a brow bar) ---
    for i in range(2):
        rim_vis = frame.get_visual(f"rim_{i}")
        ctx.check(f"rim_{i} visual exists", rim_vis is not None)

    # Verify rims are thin (X extent much smaller than Y/Z extents → ring shape)
    for i in range(2):
        rim_aabb = ctx.part_element_world_aabb(frame, elem=f"rim_{i}")
        if rim_aabb is not None:
            lo, hi = rim_aabb
            dx = hi[0] - lo[0]
            dy = hi[1] - lo[1]
            dz = hi[2] - lo[2]
            ctx.check(
                f"rim_{i} is thin (wire ring, not chunky bar)",
                dx < 0.01 and dy > 0.03 and dz > 0.03,
                details=f"dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
            )

    # Rims should be roughly circular: Y and Z extents approximately equal
    for i in range(2):
        rim_aabb = ctx.part_element_world_aabb(frame, elem=f"rim_{i}")
        if rim_aabb is not None:
            lo, hi = rim_aabb
            dy = hi[1] - lo[1]
            dz = hi[2] - lo[2]
            ratio = min(dy, dz) / max(dy, dz) if max(dy, dz) > 0 else 0
            ctx.check(
                f"rim_{i} is approximately circular (Y≈Z extents)",
                ratio > 0.85,
                details=f"dy={dy:.4f}, dz={dz:.4f}, ratio={ratio:.2f}",
            )

    # --- (3) Two circular smoky lenses ---
    for i in range(2):
        lp = object_model.get_part(f"lens_{i}")
        ctx.check(f"lens_{i} exists", lp.get_visual(f"lens_{i}") is not None)

    # Verify lenses are approximately circular (Y and Z extents ≈ 2*LENS_RADIUS)
    for i in range(2):
        lp = object_model.get_part(f"lens_{i}")
        lens_aabb = ctx.part_element_world_aabb(lp, elem=f"lens_{i}")
        if lens_aabb is not None:
            lo, hi = lens_aabb
            dy = hi[1] - lo[1]
            dz = hi[2] - lo[2]
            expected_d = 2.0 * LENS_RADIUS
            ctx.check(
                f"lens_{i} is approximately circular",
                abs(dy - expected_d) < 0.005 and abs(dz - expected_d) < 0.005,
                details=f"dy={dy:.4f}, dz={dz:.4f}, expected={expected_d:.4f}",
            )

    # Bridge connects both rim sides
    bridge_vis = frame.get_visual("bridge")
    ctx.check("bridge visual exists", bridge_vis is not None)
    for i in range(2):
        ctx.expect_overlap(
            object_model.get_part(f"lens_{i}"), frame,
            axes="y", elem_b="bridge", min_overlap=0.003,
            name=f"bridge overlaps with lens_{i} Y footprint",
        )

    # Hinge blocks and nose pads exist
    for i in range(2):
        ctx.check(
            f"hinge_block_{i} exists",
            frame.get_visual(f"hinge_block_{i}") is not None,
        )
        ctx.check(
            f"nose_pad_{i} exists",
            frame.get_visual(f"nose_pad_{i}") is not None,
        )

    # --- (4) Wire rims use polished metal material ---
    for i in range(2):
        rim_vis = frame.get_visual(f"rim_{i}")
        if rim_vis is not None:
            mat = rim_vis.material
            mat_name = mat.name if hasattr(mat, "name") else mat
            ctx.check(
                f"rim_{i} uses polished metal material",
                mat_name == "polished_metal",
                details=f"material={mat_name}",
            )

    # --- (5) No floating/disconnected parts ---
    for i in range(2):
        ctx.expect_contact(
            object_model.get_part(f"temple_{i}"), frame,
            name=f"temple_{i} connects to frame",
        )
        ctx.expect_contact(
            object_model.get_part(f"lens_{i}"), frame,
            name=f"lens_{i} connects to frame",
        )

    return ctx.report()


object_model = build_object_model()
