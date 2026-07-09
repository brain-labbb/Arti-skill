from __future__ import annotations

# Fixed picture window with a top-rail trickle vent flap.
# Variant of the side-hung wooden casement: the main sash is now a permanently
# fixed picture light captured in the frame rebate (no swing articulation).
# The only moving part is a small trickle vent flap along the top rail that
# swings outward on a horizontal revolute joint.
#
# Coordinate convention (Z up):
#   - Height runs along +Z (sill at z~0, head near the top).
#   - Width runs along X.
#   - Frame depth / glazing thickness runs along Y.  The glass plane is X-Z.
#   - The vent hinge line is horizontal along X at the top of the vent slot.
#     Positive q swings the bottom of the vent outward toward +Y.

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

WIN_W = 0.90          # overall window width (X)
WIN_H = 1.20          # overall window height (Z)
FRAME_FACE = 0.060    # outer frame member face width (in-plane, X/Z)
FRAME_DEPTH = 0.070   # outer frame depth (Y)

SILL_Z = 0.0          # bottom of the outer frame sits at z=0

# Sash: sits in the opening, slightly inset from the outer frame.
SASH_GAP = -0.001     # fixed picture glazing: sash seats into the frame rebate (1 mm overlap)
SASH_FACE = 0.055     # sash member face width
SASH_DEPTH = 0.050    # sash member depth (Y)
GLASS_T = 0.008       # tinted glass thickness (Y)
GLASS_REBATE = 0.006  # how far glass tucks under the sash lip per edge

# Opening (clear span inside the outer frame).
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = SILL_Z + FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE
OPENING_W = OPEN_X1 - OPEN_X0
OPENING_H = OPEN_Z1 - OPEN_Z0

# Sash overall size (fills the opening minus the running clearance).
SASH_W = OPENING_W - 2 * SASH_GAP
SASH_H = OPENING_H - 2 * SASH_GAP

# Sash position in world (since it is now a fixed visual on the frame part).
SASH_ORIGIN_X = OPEN_X0 + SASH_GAP       # world X of the sash local x=0 edge
SASH_BOTTOM_Z = OPEN_Z0 + SASH_GAP       # world Z of the sash local z=0 edge
SASH_TOP_Z = SASH_BOTTOM_Z + SASH_H

# Trickle vent flap (the only moving element).
VENT_W = 0.240        # vent flap width (X)
VENT_H = 0.040        # vent flap height (Z)
VENT_DEPTH = 0.010    # vent flap thickness (Y)

# Vent slot cut through the sash top rail (slightly smaller than the flap so
# the flap fully covers it when closed).
VENT_SLOT_W = 0.200
VENT_SLOT_H = 0.030

# Vent hinge sits at the top of the vent area, on the sash front face.
# In sash-local z: SASH_H - 0.005  (5 mm below the sash top outer edge, fully
# inside the top rail).
VENT_HINGE_Z_LOCAL = SASH_H - 0.005
VENT_HINGE_Z = SASH_BOTTOM_Z + VENT_HINGE_Z_LOCAL

# Slot vertical centre in sash-local coords (for the CadQuery cut).
VENT_SLOT_CZ_LOCAL = VENT_HINGE_Z_LOCAL - 0.005 - VENT_SLOT_H / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

WOOD_RGBA = (0.255, 0.145, 0.075, 1.0)    # deep brown stained wood
GLASS_RGBA = (0.16, 0.17, 0.22, 0.38)     # dark tinted glass, semi-transparent
BRASS_RGBA = (0.80, 0.62, 0.22, 1.0)      # warm gold brass hardware
DARK_RGBA = (0.06, 0.05, 0.04, 1.0)       # recessed dark opening


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _build_outer_frame_shape() -> cq.Workplane:
    """Static wood outer frame as a real hollow profile.

    A solid slab with the glass opening cut through, leaving head, sill, and
    two jambs as a continuous perimeter ring.
    """
    slab = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(
            (OPEN_X0 + OPEN_X1) / 2.0,
            0.0,
            (OPEN_Z0 + OPEN_Z1) / 2.0,
        ))
        .box(OPENING_W, FRAME_DEPTH + 0.02, OPENING_H)
    )
    return slab.cut(opening)


def _build_sash_frame_shape() -> cq.Workplane:
    """Fixed sash wood frame ring with glass opening AND vent slot cut.

    Built in sash-local coordinates:
      local x: 0 at left edge .. SASH_W at right edge
      local z: 0 at bottom .. SASH_H at top
      local y: centred on y=0, depth SASH_DEPTH
    """
    w = SASH_W
    h = SASH_H
    t = SASH_FACE
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    # Glass opening (inner void).
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, h / 2.0))
        .box(w - 2 * t, d + 0.02, h - 2 * t)
    )
    result = outer.cut(inner)

    # Vent slot through the top rail.
    slot = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, VENT_SLOT_CZ_LOCAL))
        .box(VENT_SLOT_W, d + 0.02, VENT_SLOT_H)
    )
    result = result.cut(slot)
    return result


def _build_sash_glass_shape() -> cq.Workplane:
    """Thin tinted glass pane tucked under the sash frame lip."""
    w = SASH_W
    h = SASH_H
    t = SASH_FACE
    gx0 = t - GLASS_REBATE
    gx1 = w - t + GLASS_REBATE
    gz0 = t - GLASS_REBATE
    gz1 = h - t + GLASS_REBATE
    return (
        cq.Workplane("XY")
        .transformed(offset=((gx0 + gx1) / 2.0, 0.0, (gz0 + gz1) / 2.0))
        .box(gx1 - gx0, GLASS_T, gz1 - gz0)
    )


def _build_vent_recess_shape() -> cq.Workplane:
    """Dark recess panel visible through the vent slot when the flap opens.

    Built in sash-local coordinates, positioned near the back face of the sash
    inside the vent slot void.
    """
    recess_w = VENT_SLOT_W - 0.010
    recess_h = VENT_SLOT_H + 0.002   # 2 mm taller than slot so it embeds into the sash rail above and below
    recess_d = 0.003
    recess_y = -(SASH_DEPTH / 2.0) + recess_d / 2.0 + 0.001
    return (
        cq.Workplane("XY")
        .transformed(offset=(SASH_W / 2.0, recess_y, VENT_SLOT_CZ_LOCAL))
        .box(recess_w, recess_d, recess_h)
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_picture_window_trickle_vent")

    model.material("wood", rgba=WOOD_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("brass", rgba=BRASS_RGBA)
    model.material("dark", rgba=DARK_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_outer_frame_shape(), "frame_shell"),
        material="wood",
        name="frame_shell",
    )

    # --- Fixed sash frame ring (captured in the frame rebate, no swing) ---
    frame.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sash_frame"),
        origin=Origin(xyz=(SASH_ORIGIN_X, 0.0, SASH_BOTTOM_Z)),
        material="wood",
        name="sash_frame",
    )

    # --- Fixed glass pane (dominant picture light) ---
    frame.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sash_glass"),
        origin=Origin(xyz=(SASH_ORIGIN_X, 0.0, SASH_BOTTOM_Z)),
        material="glass",
        name="sash_glass",
    )

    # --- Dark recess behind the vent slot ---
    frame.visual(
        mesh_from_cadquery(_build_vent_recess_shape(), "vent_recess"),
        origin=Origin(xyz=(SASH_ORIGIN_X, 0.0, SASH_BOTTOM_Z)),
        material="dark",
        name="vent_recess",
    )

    # --- Trickle vent flap (the only moving part) ---
    vent = model.part("vent")

    # Vent flap: a thin wooden panel.  Authored in the vent part frame whose
    # origin coincides with the articulation frame at q=0 (hinge line at the
    # top of the vent, on the sash front face).  The flap extends downward
    # (local -Z) and forward (local +Y) from the hinge.
    vent.visual(
        Box((VENT_W, VENT_DEPTH, VENT_H)),
        origin=Origin(xyz=(0.0, VENT_DEPTH / 2.0, -VENT_H / 2.0)),
        material="wood",
        name="vent_flap",
    )

    # Small brass pull-catch near the bottom of the vent flap, protruding from
    # the front face so the user can flip the vent open.
    catch_w = 0.030
    catch_d = 0.008
    catch_h = 0.010
    vent.visual(
        Box((catch_w, catch_d, catch_h)),
        origin=Origin(xyz=(
            0.0,
            VENT_DEPTH + catch_d / 2.0,
            -VENT_H + catch_h / 2.0 + 0.004,
        )),
        material="brass",
        name="vent_catch",
    )

    # Two small brass hinge knuckles at the top corners of the vent, showing
    # the pivot mounting on the sash face.  Loop-generated with name_i pattern.
    knuckle_r = 0.004
    knuckle_len = 0.016
    knuckle_x_offsets = (-VENT_W / 2.0 + 0.020, VENT_W / 2.0 - 0.020)
    for i in range(2):
        x_off = knuckle_x_offsets[i]
        # Cylinder is along local +Z; rotate 90° about Y so it lies along X
        # (the horizontal hinge axis).
        vent.visual(
            Cylinder(radius=knuckle_r, length=knuckle_len),
            origin=Origin(
                xyz=(x_off, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="brass",
            name=f"vent_knuckle_{i}",
        )

    # ----- Articulation -----
    # Trickle vent: horizontal hinge at the top edge of the vent slot, on the
    # sash front face.  Axis along +X: positive q swings the bottom of the vent
    # outward toward +Y (right-hand rule about +X: fingers curl from +Y to +Z,
    # so a point at local -Z moves toward +Y for positive q).
    model.articulation(
        "frame_to_vent",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="vent",
        origin=Origin(xyz=(0.0, SASH_DEPTH / 2.0, VENT_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=0.8),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

VENT_OPEN_POSE = 0.60  # radians, vent partially open


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    vent = object_model.get_part("vent")
    vent_joint = object_model.get_articulation("frame_to_vent")

    # --- Intentional overlaps ---

    # Glass pane is rebated under the sash frame lip (captured glass).
    ctx.allow_overlap(
        frame, frame,
        elem_a="sash_glass",
        elem_b="sash_frame",
        reason="Tinted glass pane is rebated under the sash wood frame lip so it reads as captured, not floating.",
    )
    # Fixed sash frame seats into the outer frame rebate (1 mm overlap per side).
    ctx.allow_overlap(
        frame, frame,
        elem_a="sash_frame",
        elem_b="frame_shell",
        reason="Fixed picture sash frame is captured in the outer frame rebate, with a small seating overlap at each jamb, head, and sill.",
    )
    # Dark vent recess sits inside the vent slot void within the sash frame.
    ctx.allow_overlap(
        frame, frame,
        elem_a="vent_recess",
        elem_b="sash_frame",
        reason="Dark vent recess panel is positioned inside the vent slot void within the sash top rail.",
    )
    # Brass catch is mounted on the vent flap front face.
    ctx.allow_overlap(
        vent, vent,
        elem_a="vent_catch",
        elem_b="vent_flap",
        reason="Brass pull-catch is surface-mounted on the vent flap front face.",
    )
    # Brass hinge knuckles are mounted at the vent hinge line, embedded into
    # the vent flap top edge.
    for i in range(2):
        ctx.allow_overlap(
            vent, vent,
            elem_a=f"vent_knuckle_{i}",
            elem_b="vent_flap",
            reason="Brass hinge knuckle seats into the vent flap top edge at the pivot.",
        )

    # --- Frame proportions (static root) ---
    frame_aabb = ctx.part_world_aabb(frame)
    fw = frame_aabb[1][0] - frame_aabb[0][0]
    fh = frame_aabb[1][2] - frame_aabb[0][2]

    ctx.check(
        "frame sits on the floor (z>=0)",
        frame_aabb[0][2] >= -1e-4 and frame_aabb[0][2] < 0.02,
        details=f"frame zmin={frame_aabb[0][2]:.4f}",
    )
    ctx.check(
        "window is taller than wide (stands upright)",
        fh > fw,
        details=f"frame_h={fh:.3f}, frame_w={fw:.3f}",
    )

    # --- Fixed picture glass is the dominant pane (large, inside frame) ---
    glass_aabb = ctx.part_element_world_aabb(frame, elem="sash_glass")
    if glass_aabb is not None:
        gw = glass_aabb[1][0] - glass_aabb[0][0]
        gh = glass_aabb[1][2] - glass_aabb[0][2]
        ctx.check(
            "fixed glass pane is the dominant picture light",
            gw > 0.50 and gh > 0.70,
            details=f"glass_w={gw:.3f}, glass_h={gh:.3f}",
        )

    # --- No side-hung swing articulation exists ---
    articulation_names = [a.name for a in object_model.articulations]
    ctx.check(
        "no side-hung casement swing articulation",
        "frame_to_sash" not in articulation_names,
        details=f"articulations={articulation_names}",
    )

    # --- Vent flap is small relative to the main window ---
    with ctx.pose({vent_joint: 0.0}):
        vent_aabb = ctx.part_world_aabb(vent)
        vw = vent_aabb[1][0] - vent_aabb[0][0]
        vh = vent_aabb[1][2] - vent_aabb[0][2]
        ctx.check(
            "vent flap is small relative to the window",
            vw < 0.40 and vh < 0.15,
            details=f"vent_w={vw:.3f}, vent_h={vh:.3f}",
        )
        # Vent is near the top of the window.
        vent_center_z = (vent_aabb[0][2] + vent_aabb[1][2]) / 2.0
        ctx.check(
            "vent flap is near the top of the window",
            vent_center_z > WIN_H * 0.75,
            details=f"vent_center_z={vent_center_z:.3f}, win_h={WIN_H:.3f}",
        )
        # Vent is within the frame footprint.
        ctx.expect_within(
            vent, frame,
            axes="x",
            margin=0.02,
            name="vent stays within the frame width",
        )

    # --- HERO: driving the vent joint opens the bottom outward toward +Y ---
    with ctx.pose({vent_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(vent)
        closed_max_y = closed_aabb[1][1]

    with ctx.pose({vent_joint: VENT_OPEN_POSE}):
        open_aabb = ctx.part_world_aabb(vent)
        open_max_y = open_aabb[1][1]
        ctx.check(
            "vent open pose swings outward toward +Y",
            open_max_y > closed_max_y + 0.005,
            details=f"closed max_y={closed_max_y:.4f}, open max_y={open_max_y:.4f}",
        )

    # --- Vent joint is revolute with horizontal axis (along X) ---
    axis = vent_joint.axis
    ctx.check(
        "vent joint axis is horizontal (along X)",
        abs(axis[0]) > 0.9 and abs(axis[1]) < 0.1 and abs(axis[2]) < 0.1,
        details=f"axis={axis}",
    )

    # --- Exactly one non-fixed articulation ---
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "exactly one non-fixed articulation (the vent)",
        len(non_fixed) == 1 and non_fixed[0].name == "frame_to_vent",
        details=f"non_fixed={[a.name for a in non_fixed]}",
    )

    # --- Proof checks for overlap allowances ---

    # Fixed sash stays within the outer frame footprint (rebate seating).
    ctx.expect_within(
        frame, frame,
        axes="xz",
        inner_elem="sash_frame",
        outer_elem="frame_shell",
        margin=0.005,
        name="fixed sash stays within the outer frame footprint",
    )

    # Vent recess is embedded in the sash frame at the slot edges.
    ctx.expect_within(
        frame, frame,
        axes="z",
        inner_elem="vent_recess",
        outer_elem="sash_frame",
        margin=0.002,
        name="vent recess stays within the sash frame height",
    )

    # Brass catch is mounted on the vent flap front face (contact in Y).
    ctx.expect_contact(
        vent, vent,
        elem_a="vent_catch",
        elem_b="vent_flap",
        contact_tol=0.002,
        name="brass catch contacts the vent flap at the mount",
    )

    # Hinge knuckles are seated at the vent flap top edge.
    for i in range(2):
        ctx.expect_contact(
            vent, vent,
            elem_a=f"vent_knuckle_{i}",
            elem_b="vent_flap",
            contact_tol=0.005,
            name=f"vent knuckle {i} contacts the vent flap at the pivot",
        )

    return ctx.report()


object_model = build_object_model()
