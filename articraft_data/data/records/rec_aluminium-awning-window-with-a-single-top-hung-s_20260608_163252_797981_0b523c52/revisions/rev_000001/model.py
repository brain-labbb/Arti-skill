from __future__ import annotations

# Aluminium awning window with a single top-hung sash and a bottom-rail handle.
#
# Reference (009.png): a clean, thin-framed aluminium awning window. One large
# clear-glass sash, no grille, hinged at the top rail and swung WIDE open
# (~35-45 deg) in the photo, which is the hero pose. A small lever/cam handle
# sits at the center of the sash BOTTOM rail (on the inside face).
#
# Coordinate convention:
#   +Z is up. Window stands vertically, sill at z~0. Width along X, height along
#   Z, frame depth / glass thickness / awning swing-normal along Y. Glass plane
#   is the X-Z plane. Inside (room) is -Y; the sash kicks OUTWARD toward +Y.
#
# Structure:
#   - Root "frame" (static): thin aluminium outer frame (head, sill, two jambs)
#     built as one CadQuery solid by cutting the single glass opening out of a
#     slab. No fixed lites -- the sash fills the opening.
#   - "sash" (moving): its own slim aluminium ring + one large glass pane, hinged
#     at its top rail. REVOLUTE, axis (1,0,0); positive q kicks the bottom edge
#     outward (+Y), top rail stays put. Hero open pose ~0.7 rad.
#   - The bottom-rail handle (base + lever) is a real part of the sash, mounted
#     on the inside face at the bottom-rail center, so it swings with the sash.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

WIN_W = 1.00              # overall window width (X)
WIN_H = 0.95              # overall window height (Z)
FRAME_DEPTH = 0.050       # frame profile depth (Y)
FRAME_FACE = 0.040        # thin aluminium perimeter face width (in-plane)
GLASS_T = 0.006           # glass thickness (Y)

# Sash ring (the moving leaf). Thin modern aluminium profile.
SASH_FACE = 0.034         # sash ring member face width
SASH_DEPTH = 0.038        # sash ring depth (Y), nested inside the frame rebate
SASH_CLEAR = 0.008        # running clearance, sash free edges vs the opening
HINGE_CAPTURE = 0.012     # sash top rail tucks under the frame head lip (hinge)

# Handle (bottom-rail center lever)
HANDLE_BASE_W = 0.060     # mounting plate width (X)
HANDLE_BASE_H = 0.022     # mounting plate height (Z)
HANDLE_BASE_T = 0.010     # mounting plate thickness (Y, off the glass face)
HANDLE_LEVER_LEN = 0.075  # lever bar length
HANDLE_LEVER_R = 0.0065   # lever bar radius

# Materials
ALU_RGBA = (0.74, 0.76, 0.78, 1.0)     # brushed aluminium (cool light grey)
GLASS_RGBA = (0.55, 0.62, 0.68, 0.28)  # clear glass, faint cool tint
HANDLE_RGBA = (0.30, 0.31, 0.33, 1.0)  # dark satin handle

# ---------------------------------------------------------------------------
# Opening geometry (world X-Z)
# ---------------------------------------------------------------------------

HALF_W = WIN_W / 2.0
OP_X0 = -HALF_W + FRAME_FACE
OP_X1 = HALF_W - FRAME_FACE
OP_Z0 = FRAME_FACE
OP_Z1 = WIN_H - FRAME_FACE

# Sash sits inside the opening with running clearance on its sides + bottom; its
# top rail tucks up under the frame head lip (hinge capture).
SASH_X0 = OP_X0 + SASH_CLEAR
SASH_X1 = OP_X1 - SASH_CLEAR
SASH_Z0 = OP_Z0 + SASH_CLEAR          # bottom (free) edge
SASH_Z1 = OP_Z1 + HINGE_CAPTURE       # top edge tucks under frame head
SASH_W = SASH_X1 - SASH_X0
SASH_H = SASH_Z1 - SASH_Z0
SASH_CX = (SASH_X0 + SASH_X1) / 2.0

# Glass plane and sash Y placement: sash nested toward the outside (+Y) face.
FRAME_GLASS_Y = 0.0
SASH_Y = FRAME_DEPTH / 2.0 - SASH_DEPTH / 2.0


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery)
# ---------------------------------------------------------------------------

def _frame_shape() -> cq.Workplane:
    """Thin aluminium outer frame: a slab with the single glass opening cut out.

    Leaves a true perimeter ring (head, sill, two jambs), not a box with a hole.
    """
    slab = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, WIN_H / 2.0, 0.0))
        .box(WIN_W, WIN_H, FRAME_DEPTH)
    )
    cx = (OP_X0 + OP_X1) / 2.0
    cz = (OP_Z0 + OP_Z1) / 2.0
    cut = (
        cq.Workplane("XZ")
        .transformed(offset=(cx, cz, 0.0))
        .box(OP_X1 - OP_X0, OP_Z1 - OP_Z0, FRAME_DEPTH + 0.02)
    )
    return slab.cut(cut)


def _sash_ring_shape() -> cq.Workplane:
    """Sash aluminium ring, authored in the sash-local HINGE frame.

    Local frame: hinge line (top rail) at local z=0; sash body hangs DOWN along
    local -Z to z=-SASH_H. Local x is sash width centered on 0; local y is the
    sash thickness centered on 0. A slab cut by the single glass opening leaves
    a slim ring.
    """
    w = SASH_W
    h = SASH_H
    t = SASH_FACE
    outer = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, -h / 2.0, 0.0))
        .box(w, h, SASH_DEPTH)
    )
    cut = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, -h / 2.0, 0.0))
        .box(w - 2 * t, h - 2 * t, SASH_DEPTH + 0.02)
    )
    return outer.cut(cut)


def _sash_glass_shape() -> cq.Workplane:
    """Single large glass pane for the sash, in the sash-local hinge frame."""
    w = SASH_W
    h = SASH_H
    t = SASH_FACE
    rebate = 0.008
    return (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, -h / 2.0, 0.0))
        .box(w - 2 * t + 2 * rebate, h - 2 * t + 2 * rebate, GLASS_T)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminium_awning_window")

    model.material("aluminium", rgba=ALU_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

    # --- Static root frame ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame"),
        material="aluminium",
        name="frame_shell",
    )

    # --- Moving awning sash ---
    sash = model.part("sash")
    sash.visual(
        mesh_from_cadquery(_sash_ring_shape(), "sash_ring"),
        material="aluminium",
        name="sash_ring",
    )
    sash.visual(
        mesh_from_cadquery(_sash_glass_shape(), "sash_glass"),
        material="glass",
        name="sash_glass",
    )

    # Handle at the bottom-rail center, on the inside (-Y) face, swinging with
    # the sash. In the sash-local hinge frame the bottom rail is at local
    # z=-SASH_H; the bottom-rail center is (0, 0, -SASH_H + SASH_FACE/2).
    rail_z = -SASH_H + SASH_FACE / 2.0
    inside_y = -(SASH_DEPTH / 2.0)        # inside face of the sash ring (local -Y)

    # Mounting plate seated against the rail inside face, protruding into -Y.
    base_y = inside_y - HANDLE_BASE_T / 2.0
    sash.visual(
        Box((HANDLE_BASE_W, HANDLE_BASE_T, HANDLE_BASE_H)),
        origin=Origin(xyz=(0.0, base_y, rail_z)),
        material="handle",
        name="handle_base",
    )
    # Lever bar standing off the plate further into -Y (the graspable part).
    lever_y = inside_y - HANDLE_BASE_T - HANDLE_LEVER_LEN / 2.0
    sash.visual(
        Cylinder(radius=HANDLE_LEVER_R, length=HANDLE_LEVER_LEN),
        origin=Origin(xyz=(0.0, lever_y, rail_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="handle",
        name="handle_lever",
    )

    # --- Awning articulation ---
    # Hinge line is the sash TOP rail (world z = SASH_Z1), along X. The sash body
    # hangs down along local -Z, so positive q about +X rotates it toward +Y:
    # the bottom (free) edge kicks OUTWARD (+Y), top rail stays put. axis=(1,0,0).
    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash",
        origin=Origin(xyz=(SASH_CX, SASH_Y, SASH_Z1)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=0.85),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    hinge = object_model.get_articulation("frame_to_sash")

    # Glass rebated under the sash lip (captured glass).
    ctx.allow_overlap(
        sash, sash,
        elem_a="sash_glass",
        elem_b="sash_ring",
        reason="Sash glass is rebated under the aluminium sash lip so it reads as captured, not floating.",
    )
    # Handle base seats against the sash bottom rail (mounted).
    ctx.allow_overlap(
        sash, sash,
        elem_a="handle_base",
        elem_b="sash_ring",
        reason="Handle mounting plate is seated against the sash bottom rail; this is the handle mount.",
    )
    # Hinge capture: sash top rail tucks under the frame head lip.
    ctx.allow_overlap(
        frame, sash,
        elem_a="frame_shell",
        elem_b="sash_ring",
        reason="The sash top rail is captured under the frame head lip; this is the hinge mounting, not a collision.",
    )

    # --- Closed pose (q=0): sash seated in the opening, window reads shut ---
    with ctx.pose({hinge: 0.0}):
        s_lo, s_hi = ctx.part_world_aabb(sash)
        # Sash ring is shallow in Y at closed pose (reads flat / shut). Measured
        # on the ring element, since the handle intentionally protrudes inward.
        r_lo, r_hi = ctx.part_element_world_aabb(sash, elem="sash_ring")
        sash_y_spread = r_hi[1] - r_lo[1]
        ctx.check(
            "closed sash ring is shallow in Y (reads shut)",
            sash_y_spread < 0.06,
            details=f"sash ring Y spread={sash_y_spread:.3f}",
        )
        # Sash fits inside the frame opening in X.
        ctx.expect_within(
            sash, frame, axes="x", margin=0.005,
            name="sash within frame in X",
        )
        # Hinge mount contact: sash ring meets the frame at the head.
        ctx.expect_contact(
            sash, frame,
            elem_a="sash_ring",
            elem_b="frame_shell",
            name="sash hinge edge captured by frame head",
        )
        # Handle lever sits on the inside face (negative Y), at the bottom rail.
        h_lo, h_hi = ctx.part_element_world_aabb(sash, elem="handle_lever")
        handle_z = (h_lo[2] + h_hi[2]) / 2.0
        handle_y = (h_lo[1] + h_hi[1]) / 2.0
        handle_x = (h_lo[0] + h_hi[0]) / 2.0
        sash_z_mid = (s_lo[2] + s_hi[2]) / 2.0
        ctx.check(
            "handle is on the bottom rail (below sash mid)",
            handle_z < sash_z_mid,
            details=f"handle_z={handle_z:.3f}, sash_z_mid={sash_z_mid:.3f}",
        )
        ctx.check(
            "handle is centered on the bottom rail in X",
            abs(handle_x) < 0.06,
            details=f"handle_x={handle_x:.3f}",
        )
        ctx.check(
            "handle stands off the inside (-Y) face",
            handle_y < -SASH_DEPTH / 2.0,
            details=f"handle_y={handle_y:.3f}",
        )

    # --- Frame is the static root, wider/taller than the sash, sill at floor ---
    f_lo, f_hi = ctx.part_world_aabb(frame)
    s_lo, s_hi = ctx.part_world_aabb(sash)
    frame_w = f_hi[0] - f_lo[0]
    sash_w = s_hi[0] - s_lo[0]
    ctx.check(
        "frame spans full window width and is wider than the sash",
        frame_w > WIN_W - 0.02 and frame_w > sash_w + 0.04,
        details=f"frame_w={frame_w:.3f}, sash_w={sash_w:.3f}",
    )
    ctx.check(
        "sill near z=0 (stands upright)",
        -0.02 < f_lo[2] < 0.02,
        details=f"frame zmin={f_lo[2]:.3f}",
    )

    # --- HERO: open pose ~0.7 rad swings the bottom edge WIDE out (+Y) ---
    with ctx.pose({hinge: 0.0}):
        c_lo, c_hi = ctx.part_world_aabb(sash)
    closed_y_max = c_hi[1]
    closed_bottom_z = c_lo[2]
    closed_top_z = c_hi[2]

    open_q = 0.7
    with ctx.pose({hinge: open_q}):
        o_lo, o_hi = ctx.part_world_aabb(sash)
        # Handle swings with the sash (still on the moving leaf, now lifted out).
        oh_lo, oh_hi = ctx.part_element_world_aabb(sash, elem="handle_lever")
        open_handle_y = (oh_lo[1] + oh_hi[1]) / 2.0
    open_y_max = o_hi[1]
    open_top_z = o_hi[2]

    ctx.check(
        "open sash swings bottom edge WIDE outward (+Y)",
        open_y_max > closed_y_max + 0.20,
        details=f"closed ymax={closed_y_max:.3f}, open ymax={open_y_max:.3f}",
    )
    ctx.check(
        "open sash bottom edge lifts off the closed plane",
        o_lo[2] > closed_bottom_z + 0.05,
        details=f"closed zmin={closed_bottom_z:.3f}, open zmin={o_lo[2]:.3f}",
    )
    ctx.check(
        "top hinge rail stays put when opening",
        abs(open_top_z - closed_top_z) < 0.03,
        details=f"closed top_z={closed_top_z:.3f}, open top_z={open_top_z:.3f}",
    )
    # The handle (bottom rail) travels outward with the sash as it opens.
    ctx.check(
        "handle on bottom rail swings outward with the sash",
        open_handle_y > handle_y + 0.15,
        details=f"closed handle_y={handle_y:.3f}, open handle_y={open_handle_y:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
