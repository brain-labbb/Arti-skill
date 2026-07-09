from __future__ import annotations

# Aluminum storefront double door — two narrow-stile leaves, each with a
# large single glass pane and a diagonal tubular push-bar handle.
#
# Articraft brief:
# - Object: aluminum storefront double door, ~1.70 m wide x 2.10 m tall.
# - Root/support: light anodized-aluminum frame (two side jambs + top header).
# - Parts: door_frame (root), door_leaf_0, door_leaf_1.
#   Each leaf: two vertical narrow stiles, top/bottom rails, large glass pane,
#   diagonal push-bar handle running from lower-outer to upper-inner with two
#   standoffs to the glass face.
# - Articulations: frame_to_door_leaf_0 / frame_to_door_leaf_1, both
#   REVOLUTE, vertical hinge axes at the outer jamb faces, leaves swing
#   outward (+Y from closed pose).
# - Symmetry guarantee: ONE parametric helper (_add_leaf_visuals) is
#   called with sign=+1 for door_leaf_0 and sign=-1 for door_leaf_1.
#   sign=+1: hinge edge at X=0 (world left), leaf extends +X, bar diagonal
#   from lower-hinge-side to upper-meeting-side.
#   sign=-1: mirrored. Both bars form a shallow V at the center.
# - World axes: X=width, Y=depth/swing, Z=height. Base at z=0. UP is +Z.

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dimensions (all in meters)
# ---------------------------------------------------------------------------
OPENING_W = 1.70
OPENING_H = 2.10

FRAME_FACE = 0.06       # visible frame width (side jambs in X, header in Z)
FRAME_DEPTH = 0.10      # frame depth along Y

CENTER_GAP = 0.010      # gap between meeting stiles when closed
CLEAR_W = OPENING_W - 2.0 * FRAME_FACE
LEAF_W = (CLEAR_W - CENTER_GAP) / 2.0       # ~0.785 m each leaf
LEAF_H = OPENING_H - 2.0 * FRAME_FACE       # ~1.98 m inside frame
LEAF_T = 0.045                               # leaf thickness along Y

STILE_W = 0.05          # narrow-stile vertical rails
TOP_RAIL = 0.06
BOT_RAIL = 0.16         # taller kick rail

GLASS_T = 0.010
GLASS_W = LEAF_W - 2.0 * STILE_W
GLASS_H = LEAF_H - TOP_RAIL - BOT_RAIL

BAR_R = 0.014
STANDOFF_R = 0.012
STANDOFF_LEN = 0.05     # bar proud of face surface

ALU_RGBA = (0.82, 0.83, 0.84, 1.0)
GLASS_RGBA = (0.62, 0.72, 0.78, 0.45)


# ---------------------------------------------------------------------------
# Parametric leaf geometry helpers
# ---------------------------------------------------------------------------

def _leaf_frame_body(sign: float) -> cq.Workplane:
    """Narrow-stile aluminum leaf frame: two stiles + top/bottom rails.

    Local frame: hinge edge at X=0, leaf extends sign*X, thickness centered
    at Y=0, height [0, LEAF_H].
    """
    hinge_cx = sign * STILE_W / 2.0
    free_cx = sign * (LEAF_W - STILE_W / 2.0)
    rail_cx = sign * LEAF_W / 2.0
    rail_w = LEAF_W - 2.0 * STILE_W

    hinge_stile = (
        cq.Workplane("XY")
        .box(STILE_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((hinge_cx, 0.0, 0.0))
    )
    free_stile = (
        cq.Workplane("XY")
        .box(STILE_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((free_cx, 0.0, 0.0))
    )
    bot_rail = (
        cq.Workplane("XY")
        .box(rail_w, LEAF_T, BOT_RAIL, centered=(True, True, False))
        .translate((rail_cx, 0.0, 0.0))
    )
    top_rail = (
        cq.Workplane("XY")
        .box(rail_w, LEAF_T, TOP_RAIL, centered=(True, True, False))
        .translate((rail_cx, 0.0, LEAF_H - TOP_RAIL))
    )
    return hinge_stile.union(free_stile).union(bot_rail).union(top_rail)


def _glass_pane(sign: float) -> cq.Workplane:
    """Large single glass pane filling the leaf opening."""
    glass_cx = sign * LEAF_W / 2.0
    glass_cz = BOT_RAIL + GLASS_H / 2.0
    # Oversize slightly so pane edges seat into the stile/rail rabbet.
    return (
        cq.Workplane("XY")
        .box(GLASS_W + 0.012, GLASS_T, GLASS_H + 0.012)
        .translate((glass_cx, 0.0, glass_cz))
    )


def _push_bar_mesh(sign: float, part_name: str) -> object:
    """Return a mesh for the diagonal push-bar handle using tube_from_spline_points.

    This produces a single connected triangle mesh without CadQuery union seams.
    The bar runs diagonally from lower-outer to upper-inner. Both bar endpoints
    have short standoff segments that extend slightly INTO the leaf frame body
    (past the face surface into Y>0 territory) to guarantee spatial contact
    with the frame collision hull, avoiding a disconnected-geometry warning.
    """
    front_y = -(LEAF_T / 2.0 + STANDOFF_LEN)   # bar runs at this Y
    # Embed standoff tips into the glass pane body so they overlap the glass
    # collision hull, ensuring spatial connectivity with the leaf part.
    embed_y = 0.0                                # center of the glass pane (Y=0)

    # Diagonal bar endpoints.
    low_x = sign * (STILE_W + 0.06)
    low_z = BOT_RAIL + 0.08
    high_x = sign * (LEAF_W - STILE_W - 0.04)
    high_z = BOT_RAIL + GLASS_H * 0.62

    # Tube path: embed → bar → diagonal → bar → embed.
    pts = [
        # Low standoff: embed tip into leaf face.
        (low_x, embed_y, low_z),
        (low_x, front_y, low_z),
        # Diagonal bar segment.
        (low_x + (high_x - low_x) * 0.33, front_y, low_z + (high_z - low_z) * 0.33),
        (low_x + (high_x - low_x) * 0.67, front_y, low_z + (high_z - low_z) * 0.67),
        (high_x, front_y, high_z),
        # High standoff: embed tip into leaf face.
        (high_x, embed_y, high_z),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            pts,
            radius=BAR_R,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
        f"{part_name}_push_bar",
    )


def _add_leaf_visuals(
    model: ArticulatedObject,
    part_name: str,
    sign: float,
    mat_alu: str,
    mat_glass: str,
) -> None:
    """Build and register all visuals for one leaf using the parametric helpers."""
    leaf = model.part(part_name)

    leaf.visual(
        mesh_from_cadquery(_leaf_frame_body(sign), f"{part_name}_frame"),
        material=mat_alu,
        name=f"{part_name}_frame",
    )
    leaf.visual(
        mesh_from_cadquery(_glass_pane(sign), f"{part_name}_glass"),
        material=mat_glass,
        name=f"{part_name}_glass",
    )
    bar_mesh = _push_bar_mesh(sign, part_name)
    leaf.visual(bar_mesh, origin=Origin(), material=mat_alu, name=f"{part_name}_push_bar")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_storefront_doors")

    mat_alu = model.material("anodized_aluminum", rgba=ALU_RGBA)
    mat_glass = model.material("storefront_glass", rgba=GLASS_RGBA)

    # --- Fixed anodized-aluminum frame (root): two side jambs + top header ---
    frame = model.part("door_frame")
    half_open = OPENING_W / 2.0

    frame.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(FRAME_FACE, FRAME_DEPTH, OPENING_H)
            .translate((-(half_open - FRAME_FACE / 2.0), 0.0, OPENING_H / 2.0)),
            "left_jamb",
        ),
        material=mat_alu, name="left_jamb",
    )
    frame.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(FRAME_FACE, FRAME_DEPTH, OPENING_H)
            .translate((half_open - FRAME_FACE / 2.0, 0.0, OPENING_H / 2.0)),
            "right_jamb",
        ),
        material=mat_alu, name="right_jamb",
    )
    frame.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(OPENING_W, FRAME_DEPTH, FRAME_FACE)
            .translate((0.0, 0.0, OPENING_H - FRAME_FACE / 2.0)),
            "header",
        ),
        material=mat_alu, name="header",
    )

    # --- Two glass leaves ---
    # door_leaf_0: sign=+1, hinge at left jamb inner edge, leaf extends +X.
    # door_leaf_1: sign=-1, hinge at right jamb inner edge, leaf extends -X.
    _add_leaf_visuals(model, "door_leaf_0", sign=+1.0, mat_alu=mat_alu, mat_glass=mat_glass)
    _add_leaf_visuals(model, "door_leaf_1", sign=-1.0, mat_alu=mat_alu, mat_glass=mat_glass)

    # Hinge pivot positions: at the inner face of each side jamb.
    pivot0_x = -(half_open - FRAME_FACE)   # left jamb inner face
    pivot1_x = half_open - FRAME_FACE      # right jamb inner face

    # door_leaf_0: hinge at left; positive angle about +Z swings free edge in +Y.
    # door_leaf_1: hinge at right; positive angle about -Z is symmetric outward.
    model.articulation(
        "frame_to_door_leaf_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_leaf_0"),
        origin=Origin(xyz=(pivot0_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.4),
    )
    model.articulation(
        "frame_to_door_leaf_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_leaf_1"),
        origin=Origin(xyz=(pivot1_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("door_frame")
    door0 = object_model.get_part("door_leaf_0")
    door1 = object_model.get_part("door_leaf_1")
    hinge0 = object_model.get_articulation("frame_to_door_leaf_0")
    hinge1 = object_model.get_articulation("frame_to_door_leaf_1")

    # --- Intentional embeddings ---
    for d in (door0, door1):
        ctx.allow_overlap(
            d, d,
            elem_a=f"{d.name}_glass",
            elem_b=f"{d.name}_frame",
            reason="Glass pane is glazed (lapped) into the stile/rail rabbet.",
        )
        ctx.allow_overlap(
            d, d,
            elem_a=f"{d.name}_push_bar",
            elem_b=f"{d.name}_frame",
            reason="Push-bar standoffs embed into the leaf face where fastened.",
        )
        ctx.allow_overlap(
            d, d,
            elem_a=f"{d.name}_push_bar",
            elem_b=f"{d.name}_glass",
            reason="Bar standoffs touch the glass pane face at their base.",
        )

    # --- Each leaf is appropriately sized ---
    for d in (door0, door1):
        fa = ctx.part_element_world_aabb(d, elem=f"{d.name}_frame")
        assert fa is not None
        mn, mx = fa
        w = float(mx[0] - mn[0])
        h = float(mx[2] - mn[2])
        t = float(mx[1] - mn[1])
        ctx.check(f"{d.name} leaf width plausible", 0.70 <= w <= 0.85, details=f"w={w:.3f}")
        ctx.check(f"{d.name} leaf height plausible", 1.80 <= h <= 2.10, details=f"h={h:.3f}")
        ctx.check(f"{d.name} leaf thickness plausible", 0.03 <= t <= 0.06, details=f"t={t:.3f}")
        ctx.check(f"{d.name} base at z=0", mn[2] <= 0.01, details=f"zmin={mn[2]:.4f}")

    # --- Leaf mirror-symmetry: identical frame dimensions on both leaves ---
    fa0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_frame")
    fa1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_frame")
    assert fa0 is not None and fa1 is not None
    w0 = fa0[1][0] - fa0[0][0]
    w1 = fa1[1][0] - fa1[0][0]
    h0 = fa0[1][2] - fa0[0][2]
    h1 = fa1[1][2] - fa1[0][2]
    t0 = fa0[1][1] - fa0[0][1]
    t1 = fa1[1][1] - fa1[0][1]
    ctx.check("leaf_sym_width", abs(w0 - w1) < 0.005, details=f"w0={w0:.4f} w1={w1:.4f}")
    ctx.check("leaf_sym_height", abs(h0 - h1) < 0.005, details=f"h0={h0:.4f} h1={h1:.4f}")
    ctx.check("leaf_sym_thickness", abs(t0 - t1) < 0.003, details=f"t0={t0:.4f} t1={t1:.4f}")

    # --- Each leaf has a large glass pane ---
    for d in (door0, door1):
        ga = ctx.part_element_world_aabb(d, elem=f"{d.name}_glass")
        assert ga is not None
        gw = float(ga[1][0] - ga[0][0])
        gh = float(ga[1][2] - ga[0][2])
        ctx.check(
            f"{d.name}_glass_large",
            gh > 1.0 and gw > 0.50,
            details=f"glass w={gw:.3f} h={gh:.3f}",
        )

    # Glass symmetry: same dimensions on both leaves.
    ga0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_glass")
    ga1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_glass")
    assert ga0 is not None and ga1 is not None
    gw0 = ga0[1][0] - ga0[0][0]
    gw1 = ga1[1][0] - ga1[0][0]
    gh0 = ga0[1][2] - ga0[0][2]
    gh1 = ga1[1][2] - ga1[0][2]
    ctx.check("glass_sym_width", abs(gw0 - gw1) < 0.005, details=f"gw0={gw0:.4f} gw1={gw1:.4f}")
    ctx.check("glass_sym_height", abs(gh0 - gh1) < 0.005, details=f"gh0={gh0:.4f} gh1={gh1:.4f}")

    # --- Each leaf has a diagonal push bar (spans both X and Z) ---
    for d in (door0, door1):
        ba = ctx.part_element_world_aabb(d, elem=f"{d.name}_push_bar")
        assert ba is not None
        bdx = float(ba[1][0] - ba[0][0])
        bdz = float(ba[1][2] - ba[0][2])
        ctx.check(
            f"{d.name}_bar_diagonal",
            bdx > 0.25 and bdz > 0.25,
            details=f"bar dx={bdx:.3f} dz={bdz:.3f}",
        )
        # Bar sits proud of the front (-Y) face.
        ctx.check(
            f"{d.name}_bar_proud_of_face",
            float(ba[0][1]) < -(LEAF_T / 2.0 + 0.02),
            details=f"bar minY={float(ba[0][1]):.3f}",
        )

    # Bar symmetry: same diagonal extents on both leaves.
    ba0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_push_bar")
    ba1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_push_bar")
    assert ba0 is not None and ba1 is not None
    bdx0 = ba0[1][0] - ba0[0][0]
    bdx1 = ba1[1][0] - ba1[0][0]
    bdz0 = ba0[1][2] - ba0[0][2]
    bdz1 = ba1[1][2] - ba1[0][2]
    ctx.check("bar_sym_dx", abs(bdx0 - bdx1) < 0.03, details=f"bdx0={bdx0:.4f} bdx1={bdx1:.4f}")
    ctx.check("bar_sym_dz", abs(bdz0 - bdz1) < 0.03, details=f"bdz0={bdz0:.4f} bdz1={bdz1:.4f}")

    # --- Closed pose: leaves meet at center, both on frame ---
    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        ctx.expect_gap(
            door1, door0, axis="x",
            min_gap=0.0, max_gap=0.05,
            name="leaves_meet_at_center",
        )
        ctx.expect_contact(door0, frame, name="door_leaf_0_on_frame")
        ctx.expect_contact(door1, frame, name="door_leaf_1_on_frame")

        # The two bars' high (inner/center) ends come close together in X.
        b0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_push_bar")
        b1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_push_bar")
        assert b0 is not None and b1 is not None
        ctx.check(
            "bars_form_V_near_center",
            abs(float(b0[1][0]) - float(b1[0][0])) < 0.30,
            details=f"bar0 maxX={float(b0[1][0]):.3f} bar1 minX={float(b1[0][0]):.3f}",
        )

    # --- Open pose: both leaves swing outward (+Y direction) ---
    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        c0 = ctx.part_world_aabb(door0)
        c1 = ctx.part_world_aabb(door1)
    with ctx.pose({hinge0: 1.2, hinge1: 1.2}):
        o0 = ctx.part_world_aabb(door0)
        o1 = ctx.part_world_aabb(door1)
        assert o0 and o1 and c0 and c1
        ctx.check("door_leaf_0_swings_outward",
                  float(o0[1][1]) > float(c0[1][1]) + 0.3,
                  details=f"closed={float(c0[1][1]):.3f} open={float(o0[1][1]):.3f}")
        ctx.check("door_leaf_1_swings_outward",
                  float(o1[1][1]) > float(c1[1][1]) + 0.3,
                  details=f"closed={float(c1[1][1]):.3f} open={float(o1[1][1]):.3f}")

    return ctx.report()


object_model = build_object_model()
