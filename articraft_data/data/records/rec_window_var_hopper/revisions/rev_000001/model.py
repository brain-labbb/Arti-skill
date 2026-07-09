from __future__ import annotations

# Aluminium hopper window with a single bottom-hung sash and a top-rail handle.
#
# Hopper variant of the aluminium awning window: same frame, sash, and glazing
# proportions, but the sash is hinged along its BOTTOM rail. The top edge tilts
# INWARD (-Y, toward the room) when opened. A small lever/cam handle sits at the
# center of the sash TOP rail (on the inside face), where a hopper handle
# naturally lives.
#
# Coordinate convention:
#   +Z is up. Window stands vertically, sill at z~0. Width along X, height along
#   Z, frame depth / glass thickness along Y. Glass plane is the X-Z plane.
#   Inside (room) is -Y; the sash top edge tilts INWARD toward -Y.
#
# Structure:
#   - Root "frame" (static): thin aluminium outer frame (head, sill, two jambs)
#     built as one CadQuery solid by cutting the single glass opening out of a
#     slab.
#   - "sash" (moving): its own slim aluminium ring + one large glass pane, hinged
#     at its bottom rail. REVOLUTE, axis (1,0,0); positive q tilts the top edge
#     inward (-Y), bottom rail stays put. Hero open pose ~0.7 rad.
#   - The top-rail handle (base + lever) is a real part of the sash, mounted
#     on the inside face at the top-rail center, so it swings with the sash.

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
HINGE_CAPTURE = 0.012     # sash bottom rail tucks under the frame sill lip (hinge)

# Handle (top-rail center lever)
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

# Sash sits inside the opening with running clearance on its sides + top; its
# bottom rail tucks down under the frame sill lip (hinge capture).
SASH_X0 = OP_X0 + SASH_CLEAR
SASH_X1 = OP_X1 - SASH_CLEAR
SASH_Z0 = OP_Z0 - HINGE_CAPTURE       # bottom hinge edge tucks under sill lip
SASH_Z1 = OP_Z1 - SASH_CLEAR          # top (free) edge has clearance
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
    """Sash aluminium ring, authored in the sash-local HINGE frame (bottom rail).

    Local frame: hinge line (bottom rail) at local z=0; sash body extends UP
    along local +Z to z=+SASH_H. Local x is sash width centered on 0; local y
    is the sash thickness centered on 0. A slab cut by the single glass opening
    leaves a slim ring.
    """
    w = SASH_W
    h = SASH_H
    t = SASH_FACE
    outer = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, h / 2.0, 0.0))
        .box(w, h, SASH_DEPTH)
    )
    cut = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, h / 2.0, 0.0))
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
        .transformed(offset=(0.0, h / 2.0, 0.0))
        .box(w - 2 * t + 2 * rebate, h - 2 * t + 2 * rebate, GLASS_T)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminium_hopper_window")

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

    # --- Moving hopper sash ---
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

    # Handle at the top-rail center, on the inside (-Y) face, swinging with
    # the sash. In the sash-local hinge frame the top rail is at local
    # z=+SASH_H; the top-rail center is (0, 0, SASH_H - SASH_FACE/2).
    rail_z = SASH_H - SASH_FACE / 2.0
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

    # --- Hopper articulation ---
    # Hinge line is the sash BOTTOM rail (world z = SASH_Z0), along X. The sash
    # body extends up along local +Z, so positive q about +X rotates the top
    # edge toward -Y (INWARD, into the room): the top tilts in, bottom stays
    # put. axis=(1,0,0).
    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash",
        origin=Origin(xyz=(SASH_CX, SASH_Y, SASH_Z0)),
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
    # Handle base seats against the sash top rail (mounted).
    ctx.allow_overlap(
        sash, sash,
        elem_a="handle_base",
        elem_b="sash_ring",
        reason="Handle mounting plate is seated against the sash top rail; this is the handle mount.",
    )
    # Hinge capture: sash bottom rail tucks under the frame sill lip.
    ctx.allow_overlap(
        frame, sash,
        elem_a="frame_shell",
        elem_b="sash_ring",
        reason="The sash bottom rail is captured under the frame sill lip; this is the hinge mounting, not a collision.",
    )

    # --- Closed pose (q=0): sash seated in the opening, window reads shut ---
    with ctx.pose({hinge: 0.0}):
        s_lo, s_hi = ctx.part_world_aabb(sash)
        # Sash ring is shallow in Y at closed pose (reads flat / shut).
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
        # Hinge mount contact: sash ring meets the frame at the sill.
        ctx.expect_contact(
            sash, frame,
            elem_a="sash_ring",
            elem_b="frame_shell",
            name="sash hinge edge captured by frame sill",
        )
        # Handle lever sits on the inside face (negative Y), at the top rail.
        h_lo, h_hi = ctx.part_element_world_aabb(sash, elem="handle_lever")
        handle_z = (h_lo[2] + h_hi[2]) / 2.0
        handle_y = (h_lo[1] + h_hi[1]) / 2.0
        handle_x = (h_lo[0] + h_hi[0]) / 2.0
        sash_z_mid = (s_lo[2] + s_hi[2]) / 2.0
        ctx.check(
            "handle is on the top rail (above sash mid)",
            handle_z > sash_z_mid,
            details=f"handle_z={handle_z:.3f}, sash_z_mid={sash_z_mid:.3f}",
        )
        ctx.check(
            "handle is centered on the top rail in X",
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

    # --- Hopper hinge is at the BOTTOM rail (near sill height) ---
    hinge_origin_z = hinge.origin.xyz[2] if hasattr(hinge.origin, 'xyz') else 0.0
    ctx.check(
        "hinge origin is at the bottom rail (near sill)",
        hinge_origin_z < WIN_H * 0.25,
        details=f"hinge z={hinge_origin_z:.3f}, win_h={WIN_H:.3f}",
    )

    # --- HERO: open pose ~0.7 rad tilts the top edge INWARD (-Y) ---
    with ctx.pose({hinge: 0.0}):
        c_lo, c_hi = ctx.part_world_aabb(sash)
    closed_y_min = c_lo[1]
    closed_bottom_z = c_lo[2]
    closed_top_z = c_hi[2]

    open_q = 0.7
    with ctx.pose({hinge: open_q}):
        o_lo, o_hi = ctx.part_world_aabb(sash)
        # Handle swings with the sash (still on the moving leaf, now tilted in).
        oh_lo, oh_hi = ctx.part_element_world_aabb(sash, elem="handle_lever")
        open_handle_y = (oh_lo[1] + oh_hi[1]) / 2.0
    open_y_min = o_lo[1]
    open_bottom_z = o_lo[2]

    ctx.check(
        "open sash top edge tilts INWARD (-Y, into the room)",
        open_y_min < closed_y_min - 0.10,
        details=f"closed ymin={closed_y_min:.3f}, open ymin={open_y_min:.3f}",
    )
    ctx.check(
        "bottom hinge rail stays put when opening",
        abs(open_bottom_z - closed_bottom_z) < 0.03,
        details=f"closed bottom_z={closed_bottom_z:.3f}, open bottom_z={open_bottom_z:.3f}",
    )
    # The handle (top rail) travels inward with the sash as it opens.
    ctx.check(
        "handle on top rail tilts inward with the sash",
        open_handle_y < handle_y - 0.10,
        details=f"closed handle_y={handle_y:.3f}, open handle_y={open_handle_y:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
