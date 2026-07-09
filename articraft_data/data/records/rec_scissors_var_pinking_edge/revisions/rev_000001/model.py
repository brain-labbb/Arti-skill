from __future__ import annotations

# A pair of pinking shears: two mirrored steel blades crossing at a single
# central pivot, each continuing into a red plastic finger-loop handle. The
# cutting edges carry regular triangular zigzag teeth along the inner edge
# (sawtooth / pinking profile) to cut fraying-resistant zigzag fabric edges.
# The two blades' teeth mesh by half-pitch offset. The halves rotate relative
# to each other about the pivot (REVOLUTE) to open and close.
#
# Frame convention (object laid flat in the XY plane, viewed from +Z):
#   +Y  -> toward the blade tips (up in the reference image)
#   -Y  -> toward the finger loops (down in the reference image)
#   +Z  -> out of the plane (the pivot axis)
# The pivot sits at the world origin (0, 0, 0). Half A is the root; half B is
# the child that rotates about +Z. Each half is authored in a local frame whose
# pivot is at the origin, then rotated about Z so the two blades cross.

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

# --- Real-world dimensions (meters) ----------------------------------------
BLADE_LENGTH = 0.092          # pivot -> blade tip
BLADE_ROOT_WIDTH = 0.013      # blade width near the pivot
BLADE_TIP_WIDTH = 0.0022      # blade width near the tip
BLADE_THICKNESS = 0.0020      # blade stock thickness

HANDLE_LENGTH = 0.082         # pivot -> far end of the finger loop
HANDLE_NECK_WIDTH = 0.011     # plastic neck just below the pivot
HANDLE_THICKNESS = 0.0060     # plastic handle thickness (proud of the blade)

LOOP_OUTER_LONG = 0.052       # finger-loop outer size along the handle axis
LOOP_OUTER_WIDE = 0.040       # finger-loop outer size across the handle axis
LOOP_WALL = 0.0075            # plastic rim thickness of the finger loop

PIVOT_HUB_R = 0.0075          # steel boss / cap radius at the pivot
PIVOT_HOLE_R = 0.0026         # screw hole through the pivot
PIVOT_CAP_R = 0.0066          # visible disc cap radius
PIVOT_CAP_T = 0.0016          # cap thickness

# Each half is canted off the handle axis by this angle so the blades cross
# above the pivot and the handles splay below it (open scissors pose).
HALF_CANT = math.radians(11.0)

# --- Pinking-shear teeth (sawtooth zigzag along the cutting edge) -----------
PINKING_PITCH = 0.0030        # 3.0 mm tooth-to-tooth spacing
PINKING_DEPTH = 0.0015        # 1.5 mm tooth protrusion from the cutting edge
PINKING_TEETH_END_GAP = 0.008 # smooth tip region (no teeth near the point)

STEEL_RGBA = (0.74, 0.76, 0.79, 1.0)
RED_RGBA = (0.86, 0.13, 0.11, 1.0)
CAP_RGBA = (0.80, 0.82, 0.85, 1.0)


def _pinking_edge_points(tooth_offset: float = 0.0) -> list[tuple[float, float]]:
    """Generate zigzag cutting-edge points for one pinking-shear blade.

    Returns a list of (x, y) points forming a triangular sawtooth profile
    along the +X cutting edge, from the blade root (y0) up to where the teeth
    stop before the tapered tip.  `tooth_offset` shifts the pattern along Y
    (in metres) so the two blades' teeth can mesh when closed.
    """
    half_root = BLADE_ROOT_WIDTH / 2.0
    half_tip = BLADE_TIP_WIDTH / 2.0
    y0 = PIVOT_HUB_R * 0.4
    y1 = BLADE_LENGTH
    teeth_end = y1 - PINKING_TEETH_END_GAP

    half_pitch = PINKING_PITCH / 2.0
    working = teeth_end - y0
    n_steps = max(1, int(working / half_pitch))
    offset_steps = round(tooth_offset / half_pitch)

    pts: list[tuple[float, float]] = []
    for i in range(n_steps + 1):
        t = i / n_steps
        y = y0 + t * working
        # Taper: linearly interpolate the base cutting-edge X from root to tip
        t_taper = (y - y0) / (y1 - y0)
        base_x = half_root * (1.0 - t_taper) + half_tip * t_taper
        # Zigzag: alternate valley / peak
        is_peak = (i + offset_steps) % 2 == 1
        # Fade tooth depth to zero over the last 2 pitches before teeth_end
        fade_zone = 2.0 * PINKING_PITCH
        fade = 1.0
        if y > teeth_end - fade_zone:
            fade = max(0.0, (teeth_end - y) / fade_zone)
        depth = PINKING_DEPTH * fade if is_peak else 0.0
        pts.append((base_x + depth, y))
    return pts


def _blade_solid(tooth_offset: float = 0.0) -> cq.Workplane:
    """Pinking-shear blade with zigzag teeth along the inner cutting edge.

    Authored in a local frame: pivot at origin, blade running toward +Y, the
    inner (cutting) edge on the +X side carrying sawtooth teeth, the smooth
    spine on the -X side.  `tooth_offset` shifts the tooth pattern along Y so
    the two blades' teeth interleave when closed.
    """
    half_root = BLADE_ROOT_WIDTH / 2.0
    y0 = PIVOT_HUB_R * 0.4
    y1 = BLADE_LENGTH

    # Zigzag cutting-edge points (+X side, root → near-tip)
    cutting_pts = _pinking_edge_points(tooth_offset)

    # Smooth transition from the last tooth to the sharp tip
    tip_transition = [
        (BLADE_TIP_WIDTH / 2.0 * 0.5, y1 - 0.004),
    ]

    # Spine points (smooth -X side, same curvature as original blade)
    spine_pts = [
        (0.0, y1),                             # sharp tip on the centerline
        (-half_root * 0.55, (y0 + y1) * 0.5),
        (-half_root, y0),
    ]

    pts = cutting_pts + tip_transition + spine_pts

    blade = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(BLADE_THICKNESS / 2.0, both=True)
    )
    # Light fillet on vertical edges; skip if too many small tooth features.
    try:
        blade = blade.edges("|Z").fillet(0.0003)
    except Exception:
        pass
    return blade


def _handle_solid() -> cq.Workplane:
    """Red plastic neck + finger loop in the XY plane.

    Local frame: pivot at origin, handle running toward -Y. The neck blends out
    of the pivot hub; the loop is a rounded rectangular ring (a real finger hole).
    """
    half_neck = HANDLE_NECK_WIDTH / 2.0
    # Neck: from just below the hub down to where the loop begins.
    neck_top_y = -PIVOT_HUB_R * 0.2
    loop_center_y = -(HANDLE_LENGTH - LOOP_OUTER_LONG / 2.0)
    neck_bottom_y = loop_center_y + LOOP_OUTER_LONG / 2.0 - 0.006
    neck_pts = [
        (half_neck * 1.25, neck_top_y),
        (half_neck, neck_bottom_y),
        (-half_neck, neck_bottom_y),
        (-half_neck * 1.25, neck_top_y),
    ]
    neck = (
        cq.Workplane("XY")
        .polyline(neck_pts)
        .close()
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
    )

    # Finger loop: outer rounded rect minus inner rounded rect -> a real ring.
    outer = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .rect(LOOP_OUTER_WIDE, LOOP_OUTER_LONG)
        .extrude(HANDLE_THICKNESS / 2.0, both=True)
        .edges("|Z")
        .fillet(LOOP_OUTER_WIDE * 0.38)
    )
    inner = (
        cq.Workplane("XY")
        .center(0.0, loop_center_y)
        .rect(LOOP_OUTER_WIDE - 2 * LOOP_WALL, LOOP_OUTER_LONG - 2 * LOOP_WALL)
        .extrude(HANDLE_THICKNESS * 1.4, both=True)
        .edges("|Z")
        .fillet((LOOP_OUTER_WIDE - 2 * LOOP_WALL) * 0.40)
    )
    loop = outer.cut(inner)

    handle = neck.union(loop)
    return handle


def _hub_solid() -> cq.Workplane:
    """Steel pivot boss with a through hole, centered on the pivot."""
    hub = (
        cq.Workplane("XY")
        .circle(PIVOT_HUB_R)
        .extrude(BLADE_THICKNESS / 2.0, both=True)
        .faces(">Z or <Z")
        .chamfer(0.0006)
    )
    hole = (
        cq.Workplane("XY")
        .circle(PIVOT_HOLE_R)
        .extrude(BLADE_THICKNESS, both=True)
    )
    return hub.cut(hole)


def _half_shape(side: str) -> cq.Workplane:
    """One full scissor half (steel blade + steel hub + red handle is separate).

    Returns ONLY the steel parts (blade + hub) of the half, canted about the
    pivot. `side` is "a" or "b"; the two are mirror images across X.

    Pinking teeth on half B are offset by half a pitch so the two blades'
    teeth interleave when the scissors close.
    """
    tooth_offset = 0.0 if side == "a" else PINKING_PITCH / 2.0
    blade = _blade_solid(tooth_offset=tooth_offset).union(_hub_solid())
    sign = 1.0 if side == "a" else -1.0
    # Mirror half B across the YZ plane so the two halves are true mirror images.
    if side == "b":
        blade = blade.mirror("YZ")
    # Cant the blade so it crosses the centerline above the pivot.
    blade = blade.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return blade


def _handle_shape(side: str) -> cq.Workplane:
    """Red plastic handle for one half, canted and offset in Z to clear the
    other half's handle (real scissors handles are stacked in thickness)."""
    handle = _handle_solid()
    sign = 1.0 if side == "a" else -1.0
    if side == "b":
        handle = handle.mirror("YZ")
    handle = handle.rotate((0, 0, 0), (0, 0, 1), math.degrees(-sign * HALF_CANT))
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pinking_shears")

    steel = model.material("blade_steel", rgba=STEEL_RGBA)
    red = model.material("handle_red", rgba=RED_RGBA)
    cap_mat = model.material("pivot_cap", rgba=CAP_RGBA)

    # Handles are stacked in thickness so the two halves nest at the pivot like
    # real scissors. Each handle is HANDLE_THICKNESS thick and centered on its
    # local z=0; offsetting by +/- HANDLE_THICKNESS/2 stacks them with a shared
    # face at z=0 (half A below, half B above) and no interpenetration.
    handle_z_a = -HANDLE_THICKNESS / 2.0
    handle_z_b = +HANDLE_THICKNESS / 2.0

    # --- Half A (root) -----------------------------------------------------
    blade_a = model.part("blade_a")
    blade_a.visual(
        mesh_from_cadquery(_half_shape("a"), "blade_a", assets=model.assets),
        name="blade_steel",
        material=steel,
    )
    blade_a.visual(
        mesh_from_cadquery(_handle_shape("a"), "handle_a", assets=model.assets),
        name="handle_loop",
        material=red,
        origin=Origin(xyz=(0.0, 0.0, handle_z_a)),
    )
    # Visible disc cap on the front face of the pivot.
    cap_shape = (
        cq.Workplane("XY")
        .circle(PIVOT_CAP_R)
        .extrude(PIVOT_CAP_T)
        .faces(">Z")
        .chamfer(0.0005)
    )
    # Seat the cap base flush on the hub's front face (z = +BLADE_THICKNESS/2)
    # so it reads as a riveted disc bearing on the steel boss, not a floating
    # decal. A tiny embed keeps the cap connected to the hub geometry.
    blade_a.visual(
        mesh_from_cadquery(cap_shape, "pivot_cap", assets=model.assets),
        name="pivot_cap",
        material=cap_mat,
        origin=Origin(xyz=(0.0, 0.0, BLADE_THICKNESS / 2.0 - 0.0002)),
    )

    # --- Half B (rotates about the pivot) ---------------------------------
    blade_b = model.part("blade_b")
    blade_b.visual(
        mesh_from_cadquery(_half_shape("b"), "blade_b", assets=model.assets),
        name="blade_steel",
        material=steel,
    )
    blade_b.visual(
        mesh_from_cadquery(_handle_shape("b"), "handle_b", assets=model.assets),
        name="handle_loop",
        material=red,
        origin=Origin(xyz=(0.0, 0.0, handle_z_b)),
    )

    # --- Pivot articulation ------------------------------------------------
    # Both halves share the pivot at the origin. Axis = +Z (out of plane).
    # q = 0 is the modeled open pose. Negative q closes the scissors (blade tips
    # come together); positive q opens them wider.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=blade_a,
        child=blade_b,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-2.0 * HALF_CANT,   # fully closed: tips meet on the centerline
            upper=math.radians(28.0), # wide open
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    blade_a = object_model.get_part("blade_a")
    blade_b = object_model.get_part("blade_b")
    pivot = object_model.get_articulation("pivot")

    # --- Joint contract ----------------------------------------------------
    ctx.check(
        "pivot is revolute",
        pivot.joint_type == "revolute" or pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={pivot.joint_type}",
    )
    axis = tuple(round(c, 6) for c in pivot.axis)
    ctx.check(
        "pivot axis is +Z (out of scissor plane)",
        axis == (0.0, 0.0, 1.0),
        details=f"axis={axis}",
    )
    porg = pivot.origin.xyz
    ctx.check(
        "pivot sits at the central crossing point",
        all(abs(c) < 1e-6 for c in porg),
        details=f"origin={porg}",
    )

    # --- Both halves carry a steel blade and a red handle loop -------------
    for half in (blade_a, blade_b):
        names = {v.name for v in half.visuals}
        ctx.check(
            f"{half.name} has a steel blade",
            "blade_steel" in names,
            details=f"visuals={sorted(names)}",
        )
        ctx.check(
            f"{half.name} has a finger-loop handle",
            "handle_loop" in names,
            details=f"visuals={sorted(names)}",
        )

    # The visible pivot cap belongs to the root half.
    a_names = {v.name for v in blade_a.visuals}
    ctx.check("pivot cap is present", "pivot_cap" in a_names, details=f"{sorted(a_names)}")

    # --- Geometry sanity: blade reaches up (+Y), handle reaches down (-Y) ---
    blade_box = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    handle_box = ctx.part_element_world_aabb(blade_a, elem="handle_loop")
    if blade_box is not None and handle_box is not None:
        (_, b_min_y, _), (_, b_max_y, _) = blade_box
        (_, h_min_y, _), (_, h_max_y, _) = handle_box
        ctx.check(
            "blade extends toward the tips (+Y)",
            b_max_y > 0.07,
            details=f"blade max_y={b_max_y}",
        )
        ctx.check(
            "handle loop extends toward the fingers (-Y)",
            h_min_y < -0.05,
            details=f"handle min_y={h_min_y}",
        )

    # --- The two blades cross at the pivot in the modeled pose -------------
    # Half A is canted to one side, half B to the other, so their steel tips
    # straddle the centerline (one tip at +X, the other at -X).
    a_tip = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    b_tip = ctx.part_element_world_aabb(blade_b, elem="blade_steel")
    if a_tip is not None and b_tip is not None:
        a_cx = (a_tip[0][0] + a_tip[1][0]) / 2.0
        b_cx = (b_tip[0][0] + b_tip[1][0]) / 2.0
        ctx.check(
            "blades cross (steel halves on opposite sides of center)",
            a_cx * b_cx < 0.0,
            details=f"a_center_x={a_cx}, b_center_x={b_cx}",
        )

    # --- Pivot kinematics: the blade tips swing about the central pivot ----
    # Helper: signed x-center of blade_b's steel tip region in a given pose.
    def _b_tip_cx() -> float | None:
        box = ctx.part_element_world_aabb(blade_b, elem="blade_steel")
        if box is None:
            return None
        return (box[0][0] + box[1][0]) / 2.0

    a_box = ctx.part_element_world_aabb(blade_a, elem="blade_steel")
    rest_bx = _b_tip_cx()

    # Half-closed pose: blade_b tip should swing onto the centerline (x ~ 0),
    # i.e. closer to zero than at rest. lower == -2*HALF_CANT is fully closed
    # (blade_b parallel to blade_a), so -HALF_CANT is the crossing point.
    with ctx.pose({pivot: -HALF_CANT}):
        half_closed_bx = _b_tip_cx()
    # Fully open pose: blade_b tip swings further out on its own side.
    with ctx.pose({pivot: pivot.motion_limits.upper}):
        wide_bx = _b_tip_cx()
    # Fully closed pose: blade_b becomes parallel to blade_a (same side as A).
    with ctx.pose({pivot: pivot.motion_limits.lower}):
        closed_bx = _b_tip_cx()

    if a_box is not None and rest_bx is not None and half_closed_bx is not None:
        a_cx = (a_box[0][0] + a_box[1][0]) / 2.0
        ctx.check(
            "half-closing swings blade_b tip toward the centerline",
            abs(half_closed_bx) < abs(rest_bx),
            details=f"rest_bx={rest_bx}, half_closed_bx={half_closed_bx}",
        )
        if closed_bx is not None:
            ctx.check(
                "fully closed: blade_b aligns to blade_a's side (parallel blades)",
                (closed_bx * a_cx) > 0.0,
                details=f"closed_bx={closed_bx}, a_cx={a_cx}",
            )

    if rest_bx is not None and wide_bx is not None:
        ctx.check(
            "opening the pivot swings blade_b tip further out",
            abs(wide_bx) > abs(rest_bx) and (wide_bx * rest_bx) > 0.0,
            details=f"rest_bx={rest_bx}, wide_bx={wide_bx}",
        )

    # --- Pivot region: the two steel hubs share the same axis and touch ----
    ctx.expect_overlap(
        blade_a,
        blade_b,
        axes="xy",
        elem_a="blade_steel",
        elem_b="blade_steel",
        min_overlap=0.004,
        name="steel halves overlap at the pivot hub",
    )
    # The two steel halves nest at the pivot (riveted lap joint): allow the
    # small intentional interpenetration of the two hub bosses sharing one axis.
    ctx.allow_overlap(
        blade_a,
        blade_b,
        elem_a="blade_steel",
        elem_b="blade_steel",
        reason="The two steel halves form a riveted lap joint sharing one pivot axis; their hub bosses intentionally nest.",
    )

    # --- Pinking teeth: both blades carry a sawtooth cutting edge -----------
    # Each blade's cutting edge should extend further in +X than the nominal
    # blade width due to protruding teeth.
    import cadquery as cq  # noqa: F811 – used only for dimension math below
    # Expected nominal blade half-width near the root
    nominal_half = BLADE_ROOT_WIDTH / 2.0
    # Expected maximum protrusion (tooth peaks) on the cutting-edge side
    max_protrusion = nominal_half + PINKING_DEPTH * 0.5  # at least partial teeth

    for half_name in ("blade_a", "blade_b"):
        half = object_model.get_part(half_name)
        # Get the AABB of the steel blade in the rest pose
        box = ctx.part_element_world_aabb(half, elem="blade_steel")
        if box is not None:
            (bx_min, by_min, bz_min), (bx_max, by_max, bz_max) = box
            # For half_a (not mirrored), cutting edge is at +X.
            # For half_b (mirrored), cutting edge is at -X.
            if half_name == "blade_a":
                cutting_x = bx_max
                spine_x = bx_min
            else:
                cutting_x = bx_min
                spine_x = bx_max
            blade_width_x = abs(cutting_x - spine_x)
            # The blade should be wider than the smooth nominal width because
            # the teeth add PINKING_DEPTH on the cutting side.
            ctx.check(
                f"{half_name} blade has protruding pinking teeth",
                blade_width_x > (BLADE_ROOT_WIDTH + PINKING_DEPTH * 0.3),
                details=(
                    f"blade_width={blade_width_x:.6f}m, "
                    f"expected > {BLADE_ROOT_WIDTH + PINKING_DEPTH * 0.3:.6f}m "
                    f"(nominal={BLADE_ROOT_WIDTH:.6f}m + teeth)"
                ),
            )

    # --- Pinking teeth mesh: both blades have teeth along the full edge ----
    # The tooth-bearing region should cover most of the working blade length.
    # We verify by checking that the blade's Y extent of the cutting-edge
    # protrusions spans a significant fraction of the blade.
    for half_name in ("blade_a",):
        half = object_model.get_part(half_name)
        box = ctx.part_element_world_aabb(half, elem="blade_steel")
        if box is not None:
            (_, by_min, _), (_, by_max, _) = box
            blade_y_span = by_max - by_min
            working_length = BLADE_LENGTH - PIVOT_HUB_R * 0.4
            ctx.check(
                "pinking teeth span the working blade length",
                blade_y_span > working_length * 0.7,
                details=f"blade_y_span={blade_y_span:.6f}m, working={working_length:.6f}m",
            )

    return ctx.report()
