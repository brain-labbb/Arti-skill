from __future__ import annotations

# Three-panel telescoping sliding glass door in a slim black aluminium frame.
#
# Layout (world axes):
#   X = door width (left/right along the horizontal track)
#   Y = depth/thickness (each panel sits in its own track plane)
#   Z = height (door stands on the ground plane, geometry from z=0 upward)
#
# Structure:
#   - Static root: outer aluminium perimeter frame (head, sill, two jambs)
#     plus the FIXED glass pane that occupies the left third of the opening.
#   - Two SLIDING panes (inner + outer) each carry a slim aluminium sub-frame,
#     a transparent glass infill, and a small pull handle. They telescope along
#     the horizontal track: the outer leaf travels farther than the inner one,
#     and the panes stay engaged/overlap when fully open.
#
# Articulation: two PRISMATIC joints along the horizontal track (-X opens).

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
OPENING_W = 3.00  # clear width spanned by the three panels
OPENING_H = 2.05  # clear glazed height
SILL_Z = 0.06  # bottom rail / sill height above ground
HEAD_Z = SILL_Z + OPENING_H  # top of the glazed opening

FRAME_W = 0.05  # outer frame member face width (slim black aluminium)
FRAME_D = 0.10  # outer frame depth (front-to-back), holds three track planes

# Outer opening spans X in [-X_HALF, +X_HALF]
X_HALF = OPENING_W / 2.0  # 1.50

# Each sliding panel sits in its own track plane (depth offset along Y).
PANEL_T = 0.022  # panel (frame + glass) thickness
PLANE_GAP = 0.006  # clearance between adjacent track planes
PLANE_PITCH = PANEL_T + PLANE_GAP  # center-to-center depth spacing

# Depth planes (Y) for the three panels, back -> front.
Y_FIXED = -PLANE_PITCH  # rear plane
Y_INNER = 0.0  # middle plane
Y_OUTER = +PLANE_PITCH  # front plane

# Panel widths. Sliding panels overlap the fixed pane and each other at the
# meeting stiles, as on a real telescoping slider.
STILE_W = 0.045  # vertical frame member of each sliding panel
RAIL_W = 0.045  # horizontal frame member of each sliding panel

# Panel glazed height (between sliding-panel top and bottom rails).
PANEL_H = OPENING_H  # full glazed height
PANEL_Z0 = SILL_Z  # bottom of each panel
PANEL_ZC = PANEL_Z0 + PANEL_H / 2.0  # vertical center of each panel

# Closed-position X centers and widths of the three panels.
# Fixed pane is widest (left third); inner + outer are narrower and overlap.
# Sized so the three panels (with two meeting-stile overlaps) fit inside the
# 3.00 m opening with a small clearance from the right jamb:
#   FIXED + INNER + OUTER - 2*STILE_W <= OPENING_W
# Equal-width thirds: with two meeting-stile overlaps the panel widths sum to
# OPENING_W + 2*STILE_W so the three leaves span the opening edge-to-edge.
PANEL_EQ_W = (OPENING_W + 2.0 * STILE_W) / 3.0  # 1.03
FIXED_W = PANEL_EQ_W
INNER_W = PANEL_EQ_W
OUTER_W = PANEL_EQ_W

FIXED_XC = -X_HALF + FIXED_W / 2.0  # left edge flush to jamb
# Inner panel meets the fixed pane with a small stile overlap.
INNER_XC = FIXED_XC + FIXED_W / 2.0 + INNER_W / 2.0 - STILE_W
# Outer panel meets the inner panel with a small stile overlap.
OUTER_XC = INNER_XC + INNER_W / 2.0 + OUTER_W / 2.0 - STILE_W

GLASS_INSET_T = 0.008  # glass infill thickness
HANDLE_W = 0.022
HANDLE_H = 0.16
HANDLE_D = 0.030


def _add_panel_frame(
    part,
    *,
    width: float,
    y: float,
    name_prefix: str,
    frame_mat: str,
    glass_mat: str,
) -> None:
    """Add a slim aluminium sub-frame + transparent glass infill to a panel part.

    The panel local frame is centered on the panel: x=0 is the panel center,
    z=0 is the panel vertical center, y=0 is the panel mid-thickness. The caller
    places the panel via the part origin / articulation so this stays reusable.
    """
    half_w = width / 2.0
    half_h = PANEL_H / 2.0

    # Top rail
    part.visual(
        Box((width, PANEL_T, RAIL_W)),
        origin=Origin(xyz=(0.0, 0.0, half_h - RAIL_W / 2.0)),
        material=frame_mat,
        name=f"{name_prefix}_top_rail",
    )
    # Bottom rail
    part.visual(
        Box((width, PANEL_T, RAIL_W)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + RAIL_W / 2.0)),
        material=frame_mat,
        name=f"{name_prefix}_bottom_rail",
    )
    # Left stile
    part.visual(
        Box((STILE_W, PANEL_T, PANEL_H - 2.0 * RAIL_W)),
        origin=Origin(xyz=(-half_w + STILE_W / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name=f"{name_prefix}_stile_a",
    )
    # Right stile
    part.visual(
        Box((STILE_W, PANEL_T, PANEL_H - 2.0 * RAIL_W)),
        origin=Origin(xyz=(half_w - STILE_W / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name=f"{name_prefix}_stile_b",
    )
    # Glass infill (transparent), recessed within the sub-frame opening.
    glass_w = width - 2.0 * STILE_W
    glass_h = PANEL_H - 2.0 * RAIL_W
    part.visual(
        Box((glass_w, GLASS_INSET_T, glass_h)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass_mat,
        name=f"{name_prefix}_glass",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_telescoping_sliding_door")

    # Materials -------------------------------------------------------------
    # Reference shows a uniformly slim black frame/mullion system, so all frame
    # members share one black aluminium tone (no lighter sub-frames).
    model.material("black_aluminium", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("dark_aluminium", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("handle_metal", rgba=(0.16, 0.16, 0.18, 1.0))
    # Clear glazing: nearly neutral and highly transparent, matching the
    # reference where the glass is see-through with only faint reflections.
    model.material("clear_glass", rgba=(0.78, 0.82, 0.84, 0.12))

    # ----------------------------------------------------------------------
    # Static root: outer perimeter frame + fixed pane.
    # ----------------------------------------------------------------------
    frame = model.part("frame_and_fixed_pane")

    # Outer frame members span the full opening plus the frame face width.
    outer_w = OPENING_W + 2.0 * FRAME_W
    # Head (top rail of the outer frame)
    frame.visual(
        Box((outer_w, FRAME_D, FRAME_W)),
        origin=Origin(xyz=(0.0, 0.0, HEAD_Z + FRAME_W / 2.0)),
        material="black_aluminium",
        name="frame_head",
    )
    # Sill (bottom rail of the outer frame, sitting on the ground)
    frame.visual(
        Box((outer_w, FRAME_D, SILL_Z)),
        origin=Origin(xyz=(0.0, 0.0, SILL_Z / 2.0)),
        material="black_aluminium",
        name="frame_sill",
    )
    # Left jamb
    frame.visual(
        Box((FRAME_W, FRAME_D, OPENING_H)),
        origin=Origin(xyz=(-X_HALF - FRAME_W / 2.0, 0.0, SILL_Z + OPENING_H / 2.0)),
        material="black_aluminium",
        name="frame_jamb_a",
    )
    # Right jamb
    frame.visual(
        Box((FRAME_W, FRAME_D, OPENING_H)),
        origin=Origin(xyz=(X_HALF + FRAME_W / 2.0, 0.0, SILL_Z + OPENING_H / 2.0)),
        material="black_aluminium",
        name="frame_jamb_b",
    )

    # Fixed pane sub-frame + glass, built directly in the root part at its plane.
    # We reproduce the panel members here (placed in world/root coordinates) so
    # the fixed pane reads as a glazed panel like the moving leaves.
    fx_half_w = FIXED_W / 2.0
    fx_half_h = PANEL_H / 2.0
    # Top rail
    frame.visual(
        Box((FIXED_W, PANEL_T, RAIL_W)),
        origin=Origin(xyz=(FIXED_XC, Y_FIXED, PANEL_ZC + fx_half_h - RAIL_W / 2.0)),
        material="dark_aluminium",
        name="fixed_top_rail",
    )
    # Bottom rail
    frame.visual(
        Box((FIXED_W, PANEL_T, RAIL_W)),
        origin=Origin(xyz=(FIXED_XC, Y_FIXED, PANEL_ZC - fx_half_h + RAIL_W / 2.0)),
        material="dark_aluminium",
        name="fixed_bottom_rail",
    )
    # Left stile
    frame.visual(
        Box((STILE_W, PANEL_T, PANEL_H - 2.0 * RAIL_W)),
        origin=Origin(xyz=(FIXED_XC - fx_half_w + STILE_W / 2.0, Y_FIXED, PANEL_ZC)),
        material="dark_aluminium",
        name="fixed_stile_a",
    )
    # Right stile
    frame.visual(
        Box((STILE_W, PANEL_T, PANEL_H - 2.0 * RAIL_W)),
        origin=Origin(xyz=(FIXED_XC + fx_half_w - STILE_W / 2.0, Y_FIXED, PANEL_ZC)),
        material="dark_aluminium",
        name="fixed_stile_b",
    )
    # Fixed glass infill (transparent)
    frame.visual(
        Box((FIXED_W - 2.0 * STILE_W, GLASS_INSET_T, PANEL_H - 2.0 * RAIL_W)),
        origin=Origin(xyz=(FIXED_XC, Y_FIXED, PANEL_ZC)),
        material="clear_glass",
        name="fixed_glass",
    )

    # ----------------------------------------------------------------------
    # Inner sliding pane (middle track plane).
    # Panel part frame is at the panel center; closed-position center is placed
    # via the articulation origin.
    # ----------------------------------------------------------------------
    inner = model.part("inner_sliding_pane")
    _add_panel_frame(
        inner,
        width=INNER_W,
        y=Y_INNER,
        name_prefix="inner",
        frame_mat="dark_aluminium",
        glass_mat="clear_glass",
    )
    # Pull handle on the leading (left) stile, on the front face of the panel.
    inner.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(
            xyz=(
                -INNER_W / 2.0 + STILE_W + HANDLE_W / 2.0,
                PANEL_T / 2.0 + HANDLE_D / 2.0,
                0.0,
            )
        ),
        material="handle_metal",
        name="inner_handle",
    )

    # ----------------------------------------------------------------------
    # Outer sliding pane (front track plane).
    # ----------------------------------------------------------------------
    outer = model.part("outer_sliding_pane")
    _add_panel_frame(
        outer,
        width=OUTER_W,
        y=Y_OUTER,
        name_prefix="outer",
        frame_mat="black_aluminium",
        glass_mat="clear_glass",
    )
    outer.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(
            xyz=(
                -OUTER_W / 2.0 + STILE_W + HANDLE_W / 2.0,
                PANEL_T / 2.0 + HANDLE_D / 2.0,
                0.0,
            )
        ),
        material="handle_metal",
        name="outer_handle",
    )

    # ----------------------------------------------------------------------
    # Articulations: two prismatic joints along -X (opening direction).
    #
    # At q=0 the child frame coincides with the joint frame, so the joint origin
    # is the panel's closed-position center. Positive q slides the panel toward
    # the fixed pane (-X), nesting the leaves. The outer leaf travels farther
    # than the inner leaf -> telescoping.
    # ----------------------------------------------------------------------
    # Travel budget: keep meeting-stile overlap so panes stay engaged when open.
    INNER_TRAVEL = 0.90
    OUTER_TRAVEL = 1.80

    model.articulation(
        "frame_to_inner_pane",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=inner,
        origin=Origin(xyz=(INNER_XC, Y_INNER, PANEL_ZC)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.5, lower=0.0, upper=INNER_TRAVEL
        ),
    )
    model.articulation(
        "frame_to_outer_pane",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=outer,
        origin=Origin(xyz=(OUTER_XC, Y_OUTER, PANEL_ZC)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.5, lower=0.0, upper=OUTER_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame_and_fixed_pane")
    inner = object_model.get_part("inner_sliding_pane")
    outer = object_model.get_part("outer_sliding_pane")
    j_inner = object_model.get_articulation("frame_to_inner_pane")
    j_outer = object_model.get_articulation("frame_to_outer_pane")

    # --- Stands on the ground plane, does not sink or float. ---------------
    fmin, fmax = ctx.part_world_aabb(frame)
    ctx.check(
        "frame sits on ground plane",
        abs(fmin[2]) < 0.01,
        details=f"frame z-min={fmin[2]:.4f} (expected ~0)",
    )
    ctx.check(
        "door has realistic full height",
        fmax[2] > 2.0,
        details=f"frame z-max={fmax[2]:.4f}",
    )

    # --- Panels live in distinct track planes (no coplanar collision). -----
    # Closed pose: glazing of the three panes must be separated in depth (Y),
    # which lets them slide past one another.
    ctx.expect_gap(
        inner,
        frame,
        axis="y",
        positive_elem="inner_glass",
        negative_elem="fixed_glass",
        min_gap=0.0,
        name="inner pane sits in front of fixed pane in depth",
    )
    ctx.expect_gap(
        outer,
        inner,
        axis="y",
        positive_elem="outer_glass",
        negative_elem="inner_glass",
        min_gap=0.0,
        name="outer pane sits in front of inner pane in depth",
    )

    # --- Closed pose: panels span the full opening with meeting-stile overlap.
    ctx.expect_overlap(
        inner,
        frame,
        axes="x",
        elem_a="inner_stile_a",
        elem_b="fixed_stile_b",
        min_overlap=0.0,
        name="inner pane meets fixed pane at the stile (closed)",
    )
    ctx.expect_overlap(
        outer,
        inner,
        axes="x",
        elem_a="outer_stile_a",
        elem_b="inner_stile_b",
        min_overlap=0.0,
        name="outer pane meets inner pane at the stile (closed)",
    )

    # --- Handles are mounted on (touching) their panel stiles, not floating.
    ctx.expect_contact(
        inner,
        inner,
        elem_a="inner_handle",
        elem_b="inner_stile_a",
        name="inner handle mounted on its stile",
    )
    ctx.expect_contact(
        outer,
        outer,
        elem_a="outer_handle",
        elem_b="outer_stile_a",
        name="outer handle mounted on its stile",
    )

    # --- Open pose proves the telescoping mechanism. ----------------------
    inner_closed = ctx.part_world_position(inner)
    outer_closed = ctx.part_world_position(outer)

    with ctx.pose({j_inner: 0.90, j_outer: 1.80}):
        inner_open = ctx.part_world_position(inner)
        outer_open = ctx.part_world_position(outer)
        # Both leaves slide toward the fixed side (-X).
        ctx.check(
            "inner pane opens toward fixed side",
            inner_open[0] < inner_closed[0] - 0.5,
            details=f"closed x={inner_closed[0]:.3f}, open x={inner_open[0]:.3f}",
        )
        ctx.check(
            "outer pane opens toward fixed side",
            outer_open[0] < outer_closed[0] - 0.5,
            details=f"closed x={outer_closed[0]:.3f}, open x={outer_open[0]:.3f}",
        )
        # Telescoping: outer leaf travels farther than the inner leaf.
        inner_travel = inner_closed[0] - inner_open[0]
        outer_travel = outer_closed[0] - outer_open[0]
        ctx.check(
            "outer leaf telescopes farther than inner leaf",
            outer_travel > inner_travel + 0.4,
            details=f"inner travel={inner_travel:.3f}, outer travel={outer_travel:.3f}",
        )
        # Panes stay engaged/overlap when fully open (X projection).
        ctx.expect_overlap(
            outer,
            inner,
            axes="x",
            elem_a="outer_glass",
            elem_b="inner_glass",
            min_overlap=0.10,
            name="panes remain engaged when fully open",
        )
        ctx.expect_overlap(
            inner,
            frame,
            axes="x",
            elem_a="inner_glass",
            elem_b="fixed_glass",
            min_overlap=0.10,
            name="inner pane remains engaged with fixed pane when open",
        )

    return ctx.report()


object_model = build_object_model()
