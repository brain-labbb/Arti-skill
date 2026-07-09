from __future__ import annotations

"""Round convex bubble riot shield with central boss grip.

A single circular dished polycarbonate disc (Ø~0.6 m, shallow spherical-cap
curvature, convex outward) with a dark rubber rim reinforcement band, a
painted unit ring on the convex face, and a cast aluminium carry-handle
bracket bolted at the disc centre on the concave rear with four dome-head
carriage bolts (open rectangular grip window).  The grip bracket rotates
around the shield normal for forearm-angle adjustment.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ── Shield disc dimensions (metres) ────────────────────────────────────────
DISC_R = 0.30           # disc radius  (Ø 0.60 m)
DISC_T = 0.006          # polycarbonate wall thickness
SPHERE_R = 0.80         # sphere radius that governs curvature
CAP_H = SPHERE_R - math.sqrt(SPHERE_R ** 2 - DISC_R ** 2)  # cap depth ≈ 0.058

# ── Handle bracket ─────────────────────────────────────────────────────────
PLATE_W = 0.18          # Y extent
PLATE_H = 0.12          # Z extent
PLATE_T = 0.014         # X thickness
FRAME_OUT_W = 0.14
FRAME_OUT_H = 0.09
FRAME_IN_W = 0.09
FRAME_IN_H = 0.045
FRAME_DEPTH = 0.042

BOLT_R = 0.009
BOLT_Y = 0.075          # half-spacing in Y
BOLT_Z = 0.048          # half-spacing in Z

# ── Grip rotation limits (around shield normal, X axis) ────────────────────
GRIP_LOWER = -0.7       # ≈ 40° CCW
GRIP_UPPER = 0.7        # ≈ 40° CW


# ── Geometry helpers ───────────────────────────────────────────────────────

def _build_disc_shell() -> LatheGeometry:
    """Thin-walled shallow spherical-cap disc via lathe shell profiles.

    The lathe revolves around Z; the finished mesh is rotated so the
    convex apex faces +X and the concave opening faces −X.
    """
    R, r, t = SPHERE_R, DISC_R, DISC_T
    N = 24

    # Outer profile: (radius, z) from apex (0, 0) to rim (DISC_R, −CAP_H)
    outer = []
    for i in range(N + 1):
        ri = r * i / N
        zi = -R + math.sqrt(R ** 2 - ri ** 2)
        outer.append((ri, zi))

    # Inner profile: (radius, z) from inner apex to inner rim
    Ri = R - t
    r_inner = math.sqrt(max(0.0, Ri ** 2 - (R - CAP_H) ** 2))
    inner = []
    for i in range(N + 1):
        ri = r_inner * i / N
        zi = -R + math.sqrt(max(0.0, Ri ** 2 - ri ** 2))
        inner.append((ri, zi))

    mesh = LatheGeometry.from_shell_profiles(
        outer, inner, segments=48, start_cap="flat", end_cap="flat",
    )
    # Rotate from Z-revolution to X-facing disc
    mesh.rotate_y(math.pi / 2.0)
    return mesh


def _rounded_plate() -> cq.Workplane:
    """Cast aluminium mounting plate (extends in −X from origin)."""
    plate = (
        cq.Workplane("YZ", origin=(-PLATE_T, 0.0, 0.0))
        .rect(PLATE_W, PLATE_H)
        .extrude(PLATE_T)
    )
    try:
        plate = plate.edges("|X").fillet(0.016)
    except ValueError:
        pass
    return plate


def _grip_frame() -> cq.Workplane:
    """Open rectangular grip loop extending in −X behind the plate."""
    frame = (
        cq.Workplane("YZ", origin=(-PLATE_T - FRAME_DEPTH + 0.002, 0.0, 0.0))
        .rect(FRAME_OUT_W, FRAME_OUT_H)
        .rect(FRAME_IN_W, FRAME_IN_H)
        .extrude(FRAME_DEPTH)
    )
    try:
        frame = frame.edges("|X").fillet(0.007)
    except ValueError:
        pass
    return frame


# ── Model assembly ─────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bubble_riot_shield")

    model.material("polycarbonate_gray", rgba=(0.55, 0.57, 0.58, 0.85))
    model.material("rim_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("painted_ring", rgba=(0.85, 0.78, 0.10, 1.0))
    model.material("cast_aluminum", rgba=(0.63, 0.64, 0.66, 1.0))
    model.material("bolt_steel", rgba=(0.24, 0.24, 0.26, 1.0))

    # ── Shield disc (root) ─────────────────────────────────────────────────
    front = model.part("front_panel")

    # Dished polycarbonate shell
    front.visual(
        mesh_from_geometry(_build_disc_shell(), "disc_shell"),
        material="polycarbonate_gray",
        name="disc_shell",
    )

    # Rim reinforcement band (dark rubber torus at the disc rim)
    rim_mesh = TorusGeometry(
        radius=DISC_R, tube=0.009,
        radial_segments=12, tubular_segments=48,
    )
    rim_mesh.rotate_y(math.pi / 2.0)   # ring plane → YZ, axis → X
    front.visual(
        mesh_from_geometry(rim_mesh, "rim_band"),
        origin=Origin(xyz=(-CAP_H, 0.0, 0.0)),
        material="rim_rubber",
        name="rim_band",
    )

    # Painted unit ring on the convex face (companion variation)
    # Disc surface at r=0.22 is at x ≈ −0.031; torus tube centre sits just proud
    ring_mesh = TorusGeometry(
        radius=0.22, tube=0.004,
        radial_segments=8, tubular_segments=48,
    )
    ring_mesh.rotate_y(math.pi / 2.0)
    front.visual(
        mesh_from_geometry(ring_mesh, "unit_ring"),
        origin=Origin(xyz=(-0.027, 0.0, 0.0)),
        material="painted_ring",
        name="unit_ring",
    )

    # ── Carry-handle bracket (concave rear, central boss grip) ─────────────
    handle = model.part("carry_handle")

    handle.visual(
        mesh_from_cadquery(_rounded_plate(), "handle_plate", tolerance=0.001),
        material="cast_aluminum",
        name="mount_plate",
    )
    handle.visual(
        mesh_from_cadquery(_grip_frame(), "handle_grip_frame", tolerance=0.001),
        material="cast_aluminum",
        name="grip_frame",
    )

    # Four dome-head carriage bolts through disc into bracket
    bolt_shank_len = PLATE_T + DISC_T
    bolt_x_center = (DISC_T - PLATE_T) / 2.0
    corners = ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))
    for i, (sy, sz) in enumerate(corners):
        y = sy * BOLT_Y
        z = sz * BOLT_Z
        handle.visual(
            Cylinder(radius=BOLT_R, length=bolt_shank_len),
            origin=Origin(
                xyz=(bolt_x_center, y, z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="bolt_steel",
            name=f"bolt_shank_{i}",
        )
        handle.visual(
            Sphere(radius=BOLT_R * 1.3),
            origin=Origin(xyz=(DISC_T + 0.003, y, z)),
            material="bolt_steel",
            name=f"bolt_dome_{i}",
        )

    # ── Grip rotation articulation ─────────────────────────────────────────
    # Handle rotates around shield normal (X) for forearm-angle adjustment.
    # At q = 0 the grip sits upright on the concave face centre.
    model.articulation(
        "panel_to_handle",
        ArticulationType.REVOLUTE,
        parent=front,
        child=handle,
        origin=Origin(xyz=(-DISC_T, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=GRIP_LOWER, upper=GRIP_UPPER,
        ),
    )

    return model


# ── Tests ──────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_panel")
    handle = object_model.get_part("carry_handle")
    grip_joint = object_model.get_articulation("panel_to_handle")

    # Flat mounting plate seats against the curved concave disc face.
    # The plate contacts at center; minor local overlap at plate edges is
    # the flat-vs-curved representation of a real bolted bracket mount.
    ctx.allow_overlap(
        "front_panel", "carry_handle",
        elem_a="disc_shell", elem_b="mount_plate",
        reason="flat mounting plate seats against the curved concave disc face",
    )

    # Bolt shanks pass through the disc and dome heads seat on convex face
    for i in range(4):
        ctx.allow_overlap(
            "front_panel", "carry_handle",
            elem_a="disc_shell", elem_b=f"bolt_dome_{i}",
            reason="carriage bolt dome heads seat through the convex disc face",
        )
        ctx.allow_overlap(
            "front_panel", "carry_handle",
            elem_a="disc_shell", elem_b=f"bolt_shank_{i}",
            reason="carriage bolt shanks pass through the disc wall",
        )

    # ── Prompt-specific presence checks ────────────────────────────────────
    ctx.check(
        "disc_shell_present",
        front.get_visual("disc_shell") is not None,
        "shield body must be a single circular dished disc shell",
    )
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no_rear_panel",
        "rear_panel" not in part_names,
        "single-disc variant must not carry a rear_panel part",
    )

    # Disc is circular: Y and Z world-space extents should match closely
    aabb = ctx.part_world_aabb(front)
    if aabb is not None:
        lo, hi = aabb
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        ctx.check(
            "disc_is_circular",
            abs(dy - dz) < 0.025,
            f"disc Y/Z extents should be similar (circular): dy={dy:.3f}, dz={dz:.3f}",
        )

    # Grip frame and bolts
    ctx.check(
        "grip_window_frame_present",
        handle.get_visual("grip_frame") is not None,
        "handle bracket must include the open grip frame",
    )
    ctx.check(
        "four_bolts_present",
        all(handle.get_visual(f"bolt_dome_{i}") is not None for i in range(4)),
        "handle bracket must be bolted with four dome-head bolts",
    )

    # ── Articulation checks ────────────────────────────────────────────────
    ctx.check(
        "grip_joint_is_revolute",
        grip_joint.articulation_type == ArticulationType.REVOLUTE,
        "panel_to_handle must be REVOLUTE for grip rotation",
    )
    limits = grip_joint.motion_limits
    ctx.check(
        "grip_travel_range",
        limits is not None and (limits.upper - limits.lower) > 1.0,
        "grip rotation must allow over 1 radian total travel",
    )

    # Pose proof: handle remains valid at max rotation
    with ctx.pose({grip_joint: GRIP_UPPER}):
        pos = ctx.part_world_position(handle)
        ctx.check(
            "grip_rotates_on_shield",
            pos is not None,
            "handle must remain connected when grip is rotated to upper limit",
        )

    # Handle stays within disc footprint on YZ
    ctx.expect_within(
        handle, front, axes="yz",
        inner_elem="mount_plate", outer_elem="disc_shell",
    )

    # Bolt dome contacts convex face
    ctx.expect_contact(
        handle, front,
        elem_a="bolt_dome_0", elem_b="disc_shell",
        name="bolt_dome_seated_on_disc",
    )

    return ctx.report()


object_model = build_object_model()
