from __future__ import annotations

# Weathered white-painted steel industrial awning window.
#
# Reference (006.png): a roughly square, distressed white-painted steel window.
# The opening is split by a horizontal transom into an UPPER portion holding a
# single awning sash (top-hung, tilts out from its top rail) and a LOWER portion
# split by one vertical mullion into TWO fixed glass lites. The awning sash is
# shown kicked open outward in the photo.
#
# Coordinate convention:
#   +Z is up. Window stands vertically with its sill at z~0.
#   Width runs along X, height along Z, frame depth / glass thickness / the
#   awning swing-normal run along Y. The glass plane is the X-Z plane.
#   The room/inside is -Y; the sash kicks OUTWARD toward +Y.
#
# Structure:
#   - Root "frame" (static): perimeter (head, sill, two jambs) + horizontal
#     transom + lower vertical mullion, built as one CadQuery solid by cutting
#     the three glass openings (one upper awning opening, two lower lite
#     openings) out of a slab. The two FIXED lower glass lites are part of the
#     static frame (they do not move).
#   - "awning_sash" (moving): its own slim steel ring + glass, hinged at its top
#     rail to the transom/head. REVOLUTE, axis (1,0,0); positive q kicks the
#     bottom edge outward (+Y), top rail stays put.

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
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

WIN_W = 1.10              # overall window width (X)
WIN_H = 1.05              # overall window height (Z)
FRAME_DEPTH = 0.055       # frame profile depth (Y)
FRAME_FACE = 0.060        # perimeter member face width (X or Z in-plane)
MULLION_FACE = 0.045      # transom / mullion face width
GLASS_T = 0.008           # glass thickness (Y)

# Transom: horizontal divider between upper awning portion and lower fixed pair.
# Upper portion is the taller half (awning), lower is the shorter fixed band.
TRANSOM_Z = 0.42          # center height of the horizontal transom (Z)

# Sash sizing (the moving awning leaf). It fits inside the upper opening with a
# small running clearance all round so it can swing free and reads as a separate
# leaf, not a coplanar face.
SASH_FACE = 0.040         # sash ring member face width
SASH_DEPTH = 0.040        # sash ring depth (Y), sits inside the frame rebate
SASH_CLEAR = 0.010        # running clearance between sash outer edge and opening

# Materials
STEEL_RGBA = (0.86, 0.85, 0.82, 1.0)   # weathered white-painted steel (warm grey-white)
GLASS_RGBA = (0.46, 0.52, 0.57, 0.32)  # cool grey, semi-transparent old glass

# ---------------------------------------------------------------------------
# Opening geometry (world X-Z), derived so frame members read as real members.
# ---------------------------------------------------------------------------

HALF_W = WIN_W / 2.0

# Upper awning opening (inside the perimeter, above the transom).
UP_X0 = -HALF_W + FRAME_FACE
UP_X1 = HALF_W - FRAME_FACE
UP_Z0 = TRANSOM_Z + MULLION_FACE / 2.0
UP_Z1 = WIN_H - FRAME_FACE

# Lower band: two fixed lites split by a central vertical mullion.
LO_Z0 = FRAME_FACE
LO_Z1 = TRANSOM_Z - MULLION_FACE / 2.0
LO_LEFT_X0 = -HALF_W + FRAME_FACE
LO_LEFT_X1 = -MULLION_FACE / 2.0
LO_RIGHT_X0 = MULLION_FACE / 2.0
LO_RIGHT_X1 = HALF_W - FRAME_FACE

# Sash sits inside the upper opening with running clearance on its free sides,
# but its TOP rail tucks up under the frame head lip so the hinge edge is
# physically captured by the frame (real hinge mounting, not a floating leaf).
HINGE_CAPTURE = 0.014                  # sash top rail overlaps the frame head lip
SASH_X0 = UP_X0 + SASH_CLEAR
SASH_X1 = UP_X1 - SASH_CLEAR
SASH_Z0 = UP_Z0 + SASH_CLEAR          # sash bottom edge (free edge)
SASH_Z1 = UP_Z1 + HINGE_CAPTURE       # sash top edge tucks under the frame head
SASH_W = SASH_X1 - SASH_X0
SASH_H = SASH_Z1 - SASH_Z0
SASH_CX = (SASH_X0 + SASH_X1) / 2.0   # sash center X

# The sash sits slightly proud toward +Y (outside) of the frame glass plane so
# that at the closed pose it rebates against the frame rather than sharing the
# exact same Y plane as the fixed glass.
FRAME_GLASS_Y = 0.0                    # fixed glass centered in frame depth
SASH_Y = FRAME_DEPTH / 2.0 - SASH_DEPTH / 2.0  # sash nested toward the outside (+Y) face


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery)
# ---------------------------------------------------------------------------

def _slab_with_openings() -> cq.Workplane:
    """Static steel frame: perimeter + transom + lower mullion as one solid.

    A full-face slab spanning the window outline, with the three glass openings
    (upper awning opening, lower-left lite, lower-right lite) cut away. What
    remains is a true perimeter ring + horizontal transom bar + lower vertical
    mullion -- not a box with a painted-on hole.
    """
    slab = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, WIN_H / 2.0, 0.0))
        .box(WIN_W, WIN_H, FRAME_DEPTH)
    )

    def opening(x0: float, x1: float, z0: float, z1: float) -> cq.Workplane:
        cx = (x0 + x1) / 2.0
        cz = (z0 + z1) / 2.0
        return (
            cq.Workplane("XZ")
            .transformed(offset=(cx, cz, 0.0))
            .box(x1 - x0, z1 - z0, FRAME_DEPTH + 0.02)
        )

    slab = slab.cut(opening(UP_X0, UP_X1, UP_Z0, UP_Z1))
    slab = slab.cut(opening(LO_LEFT_X0, LO_LEFT_X1, LO_Z0, LO_Z1))
    slab = slab.cut(opening(LO_RIGHT_X0, LO_RIGHT_X1, LO_Z0, LO_Z1))
    return slab


def _fixed_lite_glass() -> cq.Workplane:
    """The two FIXED lower glass lites (part of the static frame).

    Thin panes rebated under the frame lip on every edge so they read captured.
    """
    rebate = 0.008

    def pane(x0: float, x1: float, z0: float, z1: float) -> cq.Workplane:
        cx = (x0 + x1) / 2.0
        cz = (z0 + z1) / 2.0
        return (
            cq.Workplane("XZ")
            .transformed(offset=(cx, cz, FRAME_GLASS_Y))
            .box(x1 - x0, z1 - z0, GLASS_T)
        )

    left = pane(
        LO_LEFT_X0 - rebate, LO_LEFT_X1 + rebate,
        LO_Z0 - rebate, LO_Z1 + rebate,
    )
    right = pane(
        LO_RIGHT_X0 - rebate, LO_RIGHT_X1 + rebate,
        LO_Z0 - rebate, LO_Z1 + rebate,
    )
    return left.union(right)


def _sash_ring_shape() -> cq.Workplane:
    """Awning sash steel ring, authored in the sash-local HINGE frame.

    Local frame: hinge line (sash top rail) sits at local z=0. The sash body
    hangs DOWNWARD along local -Z to z = -SASH_H. Local x runs the sash width
    centered on x=0; local y is the sash thickness centered on y=0.
    A full slab cut by the single glass opening leaves a slim perimeter ring.
    """
    w = SASH_W
    h = SASH_H
    t = SASH_FACE

    outer = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, -h / 2.0, 0.0))
        .box(w, h, SASH_DEPTH)
    )
    # Glass opening inside the ring.
    cut = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, -h / 2.0, 0.0))
        .box(w - 2 * t, h - 2 * t, SASH_DEPTH + 0.02)
    )
    return outer.cut(cut)


def _sash_glass_shape() -> cq.Workplane:
    """Thin glass pane for the awning sash, in the sash-local hinge frame."""
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
    model = ArticulatedObject(name="industrial_awning_window")

    model.material("steel", rgba=STEEL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)

    # --- Static root frame (perimeter + transom + lower mullion + fixed lites) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_slab_with_openings(), "frame"),
        material="steel",
        name="frame_shell",
    )
    frame.visual(
        mesh_from_cadquery(_fixed_lite_glass(), "fixed_glass"),
        material="glass",
        name="fixed_lite_glass",
    )

    # --- Moving awning sash ---
    sash = model.part("awning_sash")
    sash.visual(
        mesh_from_cadquery(_sash_ring_shape(), "sash_ring"),
        material="steel",
        name="sash_ring",
    )
    sash.visual(
        mesh_from_cadquery(_sash_glass_shape(), "sash_glass"),
        material="glass",
        name="sash_glass",
    )

    # --- Awning articulation ---
    # Hinge line is the sash TOP rail (world z = SASH_Z1), running along X.
    # The sash body is authored hanging down along local -Z, so positive q about
    # +X rotates the local -Z body toward +Y: the bottom (free) edge kicks
    # OUTWARD (+Y) while the top rail stays on the hinge line. axis=(1,0,0).
    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="awning_sash",
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
    sash = object_model.get_part("awning_sash")
    hinge = object_model.get_articulation("frame_to_sash")

    # Glass panes are rebated under their steel lips (captured glass).
    ctx.allow_overlap(
        frame, frame,
        elem_a="fixed_lite_glass",
        elem_b="frame_shell",
        reason="Fixed lower lites are rebated under the steel frame lip so they read as captured, not floating.",
    )
    ctx.allow_overlap(
        sash, sash,
        elem_a="sash_glass",
        elem_b="sash_ring",
        reason="Sash glass is rebated under the sash steel lip so it reads as captured, not floating.",
    )
    # The sash top rail tucks up under the frame head lip: that capture is the
    # hinge mounting between the moving sash and the static frame.
    ctx.allow_overlap(
        frame, sash,
        elem_a="frame_shell",
        elem_b="sash_ring",
        reason="The awning sash top rail is captured under the frame head lip; this is the hinge mounting, not a collision.",
    )

    # --- Closed pose (q=0): sash seated in the upper opening, window reads shut ---
    with ctx.pose({hinge: 0.0}):
        sash_aabb = ctx.part_world_aabb(sash)
        frame_aabb = ctx.part_world_aabb(frame)
        s_lo, s_hi = sash_aabb
        f_lo, f_hi = frame_aabb

        # Sash sits in the upper portion of the opening (above the transom).
        sash_z_mid = (s_lo[2] + s_hi[2]) / 2.0
        ctx.check(
            "closed sash sits in upper portion",
            sash_z_mid > TRANSOM_Z,
            details=f"sash z mid={sash_z_mid:.3f}, transom={TRANSOM_Z:.3f}",
        )
        # Sash is shallow in Y at the closed pose (reads flat/shut, small spread).
        sash_y_spread = s_hi[1] - s_lo[1]
        ctx.check(
            "closed sash is shallow in Y (reads shut)",
            sash_y_spread < 0.12,
            details=f"sash Y spread={sash_y_spread:.3f}",
        )
        # Sash fits inside the frame footprint in X and Z (inside the opening).
        ctx.expect_within(
            sash, frame, axes="x", margin=0.005,
            name="sash within frame in X",
        )
        # Hinge mount: the sash top rail is captured by the frame head (contact).
        ctx.expect_contact(
            sash, frame,
            elem_a="sash_ring",
            elem_b="frame_shell",
            name="sash hinge edge captured by frame head",
        )

    # --- Frame is the static root, wider/taller than the sash ---
    frame_aabb = ctx.part_world_aabb(frame)
    sash_aabb = ctx.part_world_aabb(sash)
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    frame_h = frame_aabb[1][2] - frame_aabb[0][2]
    sash_h = sash_aabb[1][2] - sash_aabb[0][2]
    ctx.check(
        "frame spans full window width",
        frame_w > WIN_W - 0.02,
        details=f"frame_w={frame_w:.3f}",
    )
    ctx.check(
        "frame taller than the awning sash",
        frame_h > sash_h + 0.2,
        details=f"frame_h={frame_h:.3f}, sash_h={sash_h:.3f}",
    )

    # --- Sill sits at/near the floor (window stands upright, not sunk) ---
    ctx.check(
        "sill near z=0 (stands upright)",
        -0.02 < frame_aabb[0][2] < 0.02,
        details=f"frame zmin={frame_aabb[0][2]:.3f}",
    )

    # --- HERO: positive q kicks the bottom edge OUTWARD (+Y), top stays put ---
    with ctx.pose({hinge: 0.0}):
        closed = ctx.part_world_aabb(sash)
    closed_lo, closed_hi = closed
    # The free (bottom) edge in the closed pose:
    closed_bottom_z = closed_lo[2]
    closed_y_max = closed_hi[1]
    closed_top_z = closed_hi[2]

    open_q = 0.8
    with ctx.pose({hinge: open_q}):
        opened = ctx.part_world_aabb(sash)
    open_lo, open_hi = opened
    open_y_max = open_hi[1]
    open_top_z = open_hi[2]

    # Bottom edge swings out: the sash now extends much farther toward +Y.
    ctx.check(
        "open sash kicks bottom edge OUTWARD (+Y)",
        open_y_max > closed_y_max + 0.15,
        details=f"closed ymax={closed_y_max:.3f}, open ymax={open_y_max:.3f}",
    )
    # The bottom edge also rises (rotates up toward the hinge line) -> the sash
    # zmin increases as the bottom edge tilts outward and up.
    ctx.check(
        "open sash bottom edge lifts off the closed plane",
        open_lo[2] > closed_bottom_z + 0.05,
        details=f"closed zmin={closed_bottom_z:.3f}, open zmin={open_lo[2]:.3f}",
    )
    # The top (hinge) rail stays essentially put: its top Z barely changes.
    ctx.check(
        "top hinge rail stays put when opening",
        abs(open_top_z - closed_top_z) < 0.03,
        details=f"closed top_z={closed_top_z:.3f}, open top_z={open_top_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
