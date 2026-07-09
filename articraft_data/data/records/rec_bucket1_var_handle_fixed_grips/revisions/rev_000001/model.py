from __future__ import annotations

# Red painted sheet-metal fire bucket — TWO SIDE GRIP variant.
#
# Same tapered hollow shell, rolled rim, and flat bottom as the parent
# bail-handle bucket. The single swing bail is replaced by two D-loop
# side grips on +/-Y, emitted by a for-i-in-range(2) loop over a shared
# grip-geometry helper:
#   i=0  +Y side: revolute fold-flat grip (separate part, folds up flat)
#   i=1  -Y side: fixed grip (inline visual on the bucket body)
# One real non-fixed (REVOLUTE) joint remains.
#
# Coordinate convention:
#   +Z is up. The flat bottom rests on the ground at z=0.
#   The bucket is a tapered hollow revolved shell, wider at the open top.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ── bucket body dimensions (meters) ──────────────────────────────────
TOP_R = 0.140       # outer radius at the open top
BOT_R = 0.105       # outer radius at the flat bottom (tapered narrower)
BODY_H = 0.260      # bucket height (bottom z=0 → rim z=BODY_H)
WALL = 0.0016       # sheet-metal wall thickness
BOTTOM_T = 0.004    # flat bottom plate thickness

RIM_TUBE = 0.006    # rolled-rim tube (minor) radius
RIM_CENTER_R = TOP_R - RIM_TUBE * 0.4
RIM_Z = BODY_H      # rolled rim sits at the top edge

WIRE_R = 0.0028     # steel grip-wire radius
RIVET_R = 0.0035    # rivet head radius

# ── side grip dimensions ─────────────────────────────────────────────
GRIP_MOUNT_Z = BODY_H - 0.055        # grip pivot height on the wall
# Wall radius at the grip height (linear taper interpolation):
_WALL_R_AT_GRIP = BOT_R + (TOP_R - BOT_R) * GRIP_MOUNT_Z / BODY_H
GRIP_MOUNT_Y = _WALL_R_AT_GRIP + 0.013  # pivot point outside the wall

GRIP_LUG_HALF_X = 0.022   # half-span between the two lugs along X
GRIP_DEPTH = 0.045        # how far the D-loop extends from the wall
GRIP_DROP = 0.028         # how far the grip bar hangs below the pivot

GRIP_LUG_TAB_T = 0.004    # lug tab thickness along X
GRIP_LUG_TAB_H = 0.016    # lug tab height along Z

# Lug tab Y-extent: bites into the wall on the inner end, extends past the
# pivot on the outer end.
_LUG_INNER = _WALL_R_AT_GRIP - 0.005
_LUG_OUTER = GRIP_MOUNT_Y + 0.004
_LUG_REACH = _LUG_OUTER - _LUG_INNER
_LUG_CENTER = (_LUG_INNER + _LUG_OUTER) / 2.0


# ── shared geometry helpers ──────────────────────────────────────────

def _revolved_shell_mesh(top_r, bot_r, height, wall, bottom_t, name):
    """Hollow tapered bucket wall + flat bottom as one revolved thin shell."""
    outer = [
        (bot_r, 0.0),
        (bot_r + (top_r - bot_r) * 0.5, height * 0.5),
        (top_r, height),
    ]
    inner_bot_r = bot_r - wall
    inner_top_r = top_r - wall
    inner = [
        (0.0, bottom_t),
        (inner_bot_r, bottom_t),
        (inner_bot_r + (inner_top_r - inner_bot_r) * 0.5, height * 0.5),
        (inner_top_r, height),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    return mesh_from_geometry(geom, name)


def _make_dloop_grip_mesh(name):
    """Shared grip-geometry helper: D-loop side-grip wire mesh.

    Local frame: pivot axis along X at origin, grip extends along +Y,
    the grab bar hangs below the pivot axis at -Z.
    """
    lh = GRIP_LUG_HALF_X
    d = GRIP_DEPTH
    dr = GRIP_DROP
    points = [
        (+lh, 0.000, 0.000),
        (+lh, d * 0.22, -dr * 0.12),
        (+lh * 0.70, d * 0.55, -dr * 0.65),
        (+lh * 0.30, d * 0.85, -dr * 0.92),
        (0.000, d * 1.00, -dr),
        (-lh * 0.30, d * 0.85, -dr * 0.92),
        (-lh * 0.70, d * 0.55, -dr * 0.65),
        (-lh, d * 0.22, -dr * 0.12),
        (-lh, 0.000, 0.000),
    ]
    geom = tube_from_spline_points(
        points,
        radius=WIRE_R,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


# ── build ────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_bucket_side_grips")

    red_metal = model.material("red_metal", rgba=(0.62, 0.09, 0.08, 1.0))
    dark_red = model.material("dark_red", rgba=(0.50, 0.07, 0.06, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    # ── layer 1: bucket body (root) — hollow tapered shell + rolled rim ──
    bucket = model.part("bucket")

    shell_mesh = _revolved_shell_mesh(TOP_R, BOT_R, BODY_H, WALL, BOTTOM_T,
                                      "bucket_shell")
    bucket.visual(shell_mesh, material=red_metal, name="bucket_shell")

    rim_geom = TorusGeometry(radius=RIM_CENTER_R, tube=RIM_TUBE,
                             radial_segments=24, tubular_segments=64)
    rim_mesh = mesh_from_geometry(rim_geom, "rim")
    bucket.visual(rim_mesh,
                  origin=Origin(xyz=(0.0, 0.0, RIM_Z)),
                  material=dark_red, name="rolled_rim")

    bucket.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_R, length=BODY_H),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ── layer 2 & 3: two side grips (lugs, rivets, wires) ───────────────
    # Emitted by a for-i-in-range(2) loop with uniform placement.
    #   i=0: +Y fold-flat revolute grip (separate child part)
    #   i=1: -Y fixed grip (inline bucket visuals)
    fold_grip = None

    for i in range(2):
        sgn = +1 if i == 0 else -1
        side_y = sgn * GRIP_MOUNT_Y
        lug_cy = sgn * _LUG_CENTER

        # -- lug pair (±X) riveted to the bucket wall --
        for lx_sgn, lx_tag in ((+1, "r"), (-1, "l")):
            bucket.visual(
                Box((GRIP_LUG_TAB_T, _LUG_REACH, GRIP_LUG_TAB_H)),
                origin=Origin(xyz=(lx_sgn * GRIP_LUG_HALF_X, lug_cy,
                                   GRIP_MOUNT_Z)),
                material=red_metal,
                name=f"grip_lug_{lx_tag}_{i}",
            )
            bucket.visual(
                Cylinder(radius=RIVET_R, length=0.005),
                origin=Origin(xyz=(lx_sgn * GRIP_LUG_HALF_X, side_y,
                                   GRIP_MOUNT_Z),
                              rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"grip_rivet_{lx_tag}_{i}",
            )

        # -- grip wire --
        grip_mesh = _make_dloop_grip_mesh(f"grip_wire_{i}")

        if i == 0:
            # +Y side: revolute fold-flat grip (separate part)
            fold_grip = model.part("fold_grip")
            fold_grip.visual(grip_mesh, material=steel,
                             name="grip_wire_0")
            fold_grip.inertial = Inertial.from_geometry(
                Cylinder(radius=WIRE_R,
                         length=2.0 * GRIP_LUG_HALF_X + GRIP_DEPTH),
                mass=0.04,
            )
        else:
            # -Y side: fixed grip (inline on the bucket body)
            bucket.visual(
                grip_mesh,
                origin=Origin(xyz=(0.0, side_y, GRIP_MOUNT_Z),
                              rpy=(0.0, 0.0, math.pi)),
                material=steel,
                name="grip_wire_1",
            )

    # ── layer 4: single revolute articulation (fold-flat grip) ──────────
    # At q=0 the grip extends outward (+Y). Positive q rotates about +X,
    # folding the grip bar upward (+Y → +Z) until it lies flat against
    # the bucket wall.
    model.articulation(
        "bucket_to_fold_grip",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=fold_grip,
        origin=Origin(xyz=(0.0, GRIP_MOUNT_Y, GRIP_MOUNT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    return model


# ── helpers for tests ────────────────────────────────────────────────

def _aabb_center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0,
            (mn[1] + mx[1]) / 2.0,
            (mn[2] + mx[2]) / 2.0)


# ── tests ────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bucket = object_model.get_part("bucket")
    fold_grip = object_model.get_part("fold_grip")
    joint = object_model.get_articulation("bucket_to_fold_grip")

    # ── intentional-overlap allowances (scoped) ─────────────────────
    # Lug tabs are riveted onto the bucket wall / near the rim.
    for i in range(2):
        for lx in ("r", "l"):
            ctx.allow_overlap(
                bucket, bucket,
                elem_a=f"grip_lug_{lx}_{i}", elem_b="bucket_shell",
                reason=f"Grip {i} {lx}-lug tab is riveted onto the bucket wall.",
            )
            ctx.allow_overlap(
                bucket, bucket,
                elem_a=f"grip_rivet_{lx}_{i}", elem_b=f"grip_lug_{lx}_{i}",
                reason=f"Rivet is set through grip {i} {lx}-lug tab.",
            )
    # Fold-grip wire ends seat into their lugs / rivets.
    for lx in ("r", "l"):
        ctx.allow_overlap(
            fold_grip, bucket,
            elem_a="grip_wire_0", elem_b=f"grip_lug_{lx}_0",
            reason="Fold-grip wire end seats into the lug.",
        )
        ctx.allow_overlap(
            fold_grip, bucket,
            elem_a="grip_wire_0", elem_b=f"grip_rivet_{lx}_0",
            reason="Fold-grip wire end overlaps the rivet head.",
        )
    # Fixed-grip wire ends seat into their lugs / rivets (all on bucket).
    for lx in ("r", "l"):
        ctx.allow_overlap(
            bucket, bucket,
            elem_a="grip_wire_1", elem_b=f"grip_lug_{lx}_1",
            reason="Fixed-grip wire end seats into the lug.",
        )
        ctx.allow_overlap(
            bucket, bucket,
            elem_a="grip_wire_1", elem_b=f"grip_rivet_{lx}_1",
            reason="Fixed-grip wire end overlaps the rivet head.",
        )

    # ── body checks (preserved from parent) ─────────────────────────
    shell_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")
    ctx.check(
        "flat bottom rests on the ground at z~0",
        abs(shell_aabb[0][2]) < 0.002,
        details=f"shell_min_z={shell_aabb[0][2]:.4f}",
    )
    ctx.check(
        "bucket height ~0.26 m",
        abs((shell_aabb[1][2] - shell_aabb[0][2]) - BODY_H) < 0.01,
        details=f"shell_h={shell_aabb[1][2] - shell_aabb[0][2]:.4f}",
    )
    rim_aabb = ctx.part_element_world_aabb(bucket, elem="rolled_rim")
    top_w = rim_aabb[1][0] - rim_aabb[0][0]
    ctx.check(
        "tapered: open top wider than flat bottom",
        top_w > 2.0 * BOT_R + 0.03,
        details=f"top_w={top_w:.4f}, bottom_d={2 * BOT_R:.4f}",
    )
    ctx.check(
        "body is hollow (thin wall, open interior)",
        WALL < TOP_R * 0.05 and BOTTOM_T < BODY_H * 0.1,
        details=f"wall={WALL}, bottom_t={BOTTOM_T}, top_r={TOP_R}",
    )

    # ── two symmetric side grips on +/-Y ────────────────────────────
    gw0_aabb = ctx.part_element_world_aabb(fold_grip, elem="grip_wire_0")
    gw1_aabb = ctx.part_element_world_aabb(bucket, elem="grip_wire_1")
    c0 = _aabb_center(gw0_aabb)
    c1 = _aabb_center(gw1_aabb)

    ctx.check(
        "two side grips on opposite +/-Y sides",
        c0[1] > 0.05 and c1[1] < -0.05,
        details=f"grip_0_y={c0[1]:.4f}, grip_1_y={c1[1]:.4f}",
    )
    ctx.check(
        "side grips symmetric about Y=0",
        abs(abs(c0[1]) - abs(c1[1])) < 0.012,
        details=f"|grip_0_y|={abs(c0[1]):.4f}, |grip_1_y|={abs(c1[1]):.4f}",
    )
    ctx.check(
        "both grips at similar height",
        abs(c0[2] - c1[2]) < 0.012,
        details=f"grip_0_z={c0[2]:.4f}, grip_1_z={c1[2]:.4f}",
    )
    # Each grip extends meaningfully outward from the wall.
    ctx.check(
        "+Y grip extends outside the bucket wall",
        gw0_aabb[1][1] > TOP_R + 0.02,
        details=f"grip_0_max_y={gw0_aabb[1][1]:.4f}, top_r={TOP_R}",
    )
    ctx.check(
        "-Y grip extends outside the bucket wall",
        gw1_aabb[0][1] < -(TOP_R + 0.02),
        details=f"grip_1_min_y={gw1_aabb[0][1]:.4f}",
    )

    # ── fixed grip does not articulate ──────────────────────────────
    # The fixed grip wire (grip_wire_1) is an inline visual on the bucket
    # part, not a separate part. No articulation moves the bucket.
    child_names = [a.child for a in object_model.articulations]
    ctx.check(
        "bucket is not a joint child (fixed grip is structural)",
        "bucket" not in child_names,
        details=f"child_names={child_names}",
    )
    ctx.check(
        "fixed grip is a bucket visual, not a separate part",
        bucket.get_visual("grip_wire_1") is not None,
        details="grip_wire_1 not found on bucket part",
    )

    # ── exactly one real non-fixed (REVOLUTE) joint ─────────────────
    all_arts = list(object_model.articulations)
    revolute_arts = [
        a for a in all_arts
        if str(a.articulation_type).upper().endswith("REVOLUTE")
    ]
    ctx.check(
        "exactly one articulation in the model",
        len(all_arts) == 1,
        details=f"count={len(all_arts)}, names={[a.name for a in all_arts]}",
    )
    ctx.check(
        "the single articulation is REVOLUTE",
        len(revolute_arts) == 1,
        details=f"revolute_count={len(revolute_arts)}",
    )

    # Joint origin sits at the fold-grip pivot on the bucket wall.
    jo = joint.origin.xyz
    ctx.check(
        "joint origin at the fold-grip pivot point",
        abs(jo[0]) < 1e-5
        and abs(jo[1] - GRIP_MOUNT_Y) < 0.005
        and abs(jo[2] - GRIP_MOUNT_Z) < 0.005,
        details=f"origin={jo}, expected_y={GRIP_MOUNT_Y}, expected_z={GRIP_MOUNT_Z}",
    )
    ax = joint.axis
    ctx.check(
        "fold-grip axis is horizontal X (folds up against wall)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 0.02 and abs(ax[2]) < 0.02,
        details=f"axis={ax}",
    )

    # ── decisive pose: fold grip swings upward when rotated ─────────
    rest_center = _aabb_center(
        ctx.part_element_world_aabb(fold_grip, elem="grip_wire_0")
    )
    with ctx.pose({joint: joint.motion_limits.upper}):
        folded_center = _aabb_center(
            ctx.part_element_world_aabb(fold_grip, elem="grip_wire_0")
        )
    ctx.check(
        "fold grip bar moves upward when rotated to upper limit",
        folded_center[2] > rest_center[2] + 0.03,
        details=f"rest_z={rest_center[2]:.4f}, folded_z={folded_center[2]:.4f}",
    )
    ctx.check(
        "fold grip bar retracts inward when folded",
        folded_center[1] < rest_center[1] - 0.005,
        details=f"rest_y={rest_center[1]:.4f}, folded_y={folded_center[1]:.4f}",
    )

    # ── colors: body red, wire steel ────────────────────────────────
    shell_rgba = bucket.get_visual("bucket_shell").material.rgba
    wire_rgba = fold_grip.get_visual("grip_wire_0").material.rgba
    ctx.check(
        "bucket body is red",
        shell_rgba[0] > 0.4 and shell_rgba[1] < 0.25 and shell_rgba[2] < 0.25,
        details=f"shell_rgba={shell_rgba}",
    )
    ctx.check(
        "grip wire is steel-gray",
        min(wire_rgba[:3]) > 0.55
        and max(wire_rgba[:3]) - min(wire_rgba[:3]) < 0.2,
        details=f"wire_rgba={wire_rgba}",
    )

    return ctx.report()


object_model = build_object_model()
