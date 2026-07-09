from __future__ import annotations

"""Matchbook with a folded cardboard cover and paper match comb.

Layout (world frame, Z up, matchbook standing on the ground plane z=0):
- The cream cardboard cover is the root part. It is a folded shell: a tall
  back panel (0.055 m wide x 0.035 m tall), a short front panel (0.018 m
  tall) parallel to the back panel separated by a 0.010 m pocket, and a
  thin bottom spine connecting them. A reddish-brown stippled striker
  strip sits on the outside of the front panel.
- Inside the pocket a paper base strip is glued across the bottom; a comb
  of 10 thin flat paper match tabs stands up from it, each capped with a
  dark reddish-brown chemical head.
- The top flap is a cardboard panel hinged at the visible top fold line of
  the back panel with a revolute joint (axis along the long axis). The
  rest pose (q = 0) shows the flap open about 70 degrees so the matches
  are exposed; the lower limit closes the flap, the upper limit allows it
  to open further.
- Two loose paper matches lie on the ground in front (+Y) of the book.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ----------------------------------------------------------------- constants
COVER_W = 0.055       # X, long axis
BACK_H = 0.035        # Z, back panel height
FRONT_H = 0.018       # Z, front panel height (shorter)
POCKET_D = 0.010      # Y, pocket depth between panels
T = 0.0008            # cardboard wall thickness
FLAP_H = 0.020        # Z, flap panel height

# Paper match tabs (standing up in the comb)
TAB_W = 0.003         # X, tab width
TAB_T = 0.0005        # Y, tab thickness
TAB_H = 0.025         # Z, tab height
HEAD_H = 0.005        # Z, chemical head height
HEAD_EMBED = 0.001    # Z, head embeds slightly into tab top

# Base strip inside pocket
BASE_W = 0.048        # X, spans most of the cover width
BASE_T = 0.001        # Y
BASE_H = 0.003        # Z

N_MATCHES = 10

# Loose paper matches on the ground
LOOSE_L = 0.030       # length (lies along local X)
LOOSE_W = 0.003       # width (local Y)
LOOSE_T = 0.0005      # thickness (local Z, vertical when flat)
LOOSE_HEAD_L = 0.006  # head length along the stick

# Articulation. The flap is hinged at the top fold line of the back panel and
# extends UPWARD from there (a continuation of the back panel), so folding it
# never sweeps the panel through the back wall. Rotation is about +X: positive
# leans the flap back away from the matches (open), negative folds it forward
# and down over the front (closed).
BACK_TILT = math.radians(25)        # rest pose: flap leans back, matches exposed
FLAP_CLOSED_Q = math.radians(-175)  # joint value that folds the flap shut over the front
FLAP_OPEN_Q = math.radians(30)      # joint value for fully open (a little further back)

# ----------------------------------------------------------------- materials
CREAM = Material(name="cover_cardboard", rgba=(0.89, 0.84, 0.71, 1.0))
STRIKER = Material(name="striker_stipple", rgba=(0.66, 0.36, 0.28, 1.0))
MATCH_PAPER = Material(name="match_paper", rgba=(0.93, 0.91, 0.86, 1.0))
HEAD_BROWN = Material(name="head_brown", rgba=(0.38, 0.13, 0.08, 1.0))


# ----------------------------------------------------------- geometry helpers
def _add_paper_match_standing(part, name, xc, yc, z_base):
    """Shared helper: one standing paper match tab + head on the given part."""
    tab_zc = z_base + TAB_H / 2.0
    head_zc = z_base + TAB_H + HEAD_H / 2.0 - HEAD_EMBED
    part.visual(
        Box((TAB_W, TAB_T, TAB_H)),
        origin=Origin(xyz=(xc, yc, tab_zc)),
        material=MATCH_PAPER,
        name=f"{name}_tab",
    )
    part.visual(
        Box((TAB_W, TAB_T, HEAD_H)),
        origin=Origin(xyz=(xc, yc, head_zc)),
        material=HEAD_BROWN,
        name=f"{name}_head",
    )


def _add_paper_match_flat(part, name, tab_origin, head_origin):
    """Shared helper: one paper match lying flat on the given part."""
    part.visual(
        Box((LOOSE_L, LOOSE_W, LOOSE_T)),
        origin=tab_origin,
        material=MATCH_PAPER,
        name=f"{name}_tab",
    )
    part.visual(
        Box((LOOSE_HEAD_L, LOOSE_W, LOOSE_T + 0.0002)),
        origin=head_origin,
        material=HEAD_BROWN,
        name=f"{name}_head",
    )


# --------------------------------------------------------------------- build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="matchbook")

    # ------------------------------------------------------- cover (root)
    cover = model.part("cover")

    # Back panel: tall vertical thin panel
    cover.visual(
        Box((COVER_W, T, BACK_H)),
        origin=Origin(xyz=(0.0, -T / 2.0, BACK_H / 2.0)),
        material=CREAM,
        name="back_panel",
    )

    # Front panel: shorter, parallel, offset in +Y by the pocket depth
    cover.visual(
        Box((COVER_W, T, FRONT_H)),
        origin=Origin(xyz=(0.0, POCKET_D + T / 2.0, FRONT_H / 2.0)),
        material=CREAM,
        name="front_panel",
    )

    # Bottom spine connecting the two panels
    cover.visual(
        Box((COVER_W, POCKET_D, T)),
        origin=Origin(xyz=(0.0, POCKET_D / 2.0, T / 2.0)),
        material=CREAM,
        name="spine",
    )

    # Striker strip on the outside (+Y face) of the front panel
    cover.visual(
        Box((0.050, 0.0006, 0.014)),
        origin=Origin(xyz=(0.0, POCKET_D + T + 0.0001, FRONT_H / 2.0)),
        material=STRIKER,
        name="striker",
    )

    # Base strip glued across the bottom inside the pocket
    cover.visual(
        Box((BASE_W, BASE_T, BASE_H)),
        origin=Origin(xyz=(0.0, POCKET_D / 2.0, T + BASE_H / 2.0)),
        material=MATCH_PAPER,
        name="base_strip",
    )

    # Paper match comb: 10 tabs standing up from the base strip
    pitch = BASE_W / N_MATCHES
    tab_z_base = T + BASE_H       # bottom of tabs on top of base strip
    tab_y = POCKET_D / 2.0        # centered in the pocket
    for i in range(N_MATCHES):
        xc = -BASE_W / 2.0 + pitch / 2.0 + i * pitch
        _add_paper_match_standing(cover, f"match_{i}", xc, tab_y, tab_z_base)

    # ----------------------------------------------------------- top flap
    flap = model.part("flap")

    # The flap panel extends UP from the hinge (a continuation of the back
    # panel) in the flap part frame, sitting just in front of the back panel
    # plane (y in [0, T]). Because the panel lives above the pivot, folding it
    # forward to close never sweeps it through the back panel below the hinge.
    flap.visual(
        Box((COVER_W, T, FLAP_H)),
        origin=Origin(xyz=(0.0, T / 2.0, FLAP_H / 2.0)),
        material=CREAM,
        name="flap_panel",
    )

    # Revolute hinge at the top fold line (front-top edge of the back panel).
    # axis (1,0,0): positive q leans the flap further back, negative q folds it
    # forward and down over the front. The rpy offset encodes the rest lean-back
    # so q = 0 is the displayed open pose.
    model.articulation(
        "cover_to_flap",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=flap,
        origin=Origin(
            xyz=(0.0, 0.0, BACK_H),
            rpy=(BACK_TILT, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=FLAP_CLOSED_Q,  # folded forward and down over the front: closed
            upper=FLAP_OPEN_Q,    # leans a little further back: fully open
        ),
    )

    # ------------------------------------------ loose matches on the ground
    ground_z = LOOSE_T / 2.0  # tabs lie flat on z = 0
    placements = (
        (0.000, 0.030, 0.10),   # (x, y, yaw)
        (0.014, 0.044, -0.30),
    )
    for i, (cx, cy, yaw) in enumerate(placements):
        gm = model.part(f"ground_match_{i}")
        _add_paper_match_flat(
            gm,
            f"ground_{i}",
            tab_origin=Origin(xyz=(0.0, 0.0, 0.0)),
            head_origin=Origin(
                xyz=(LOOSE_L / 2.0 + LOOSE_HEAD_L / 2.0 - 0.001, 0.0, 0.0001),
            ),
        )
        model.articulation(
            f"cover_to_ground_{i}",
            ArticulationType.FIXED,
            parent=cover,
            child=gm,
            origin=Origin(xyz=(cx, cy, ground_z), rpy=(0.0, 0.0, yaw)),
        )

    return model


# --------------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cover = object_model.get_part("cover")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("cover_to_flap")
    ground_matches = [
        object_model.get_part(f"ground_match_{i}") for i in range(2)
    ]

    # Loose matches rest on the ground in front of the matchbook.
    for gm in ground_matches:
        ctx.allow_isolated_part(
            gm,
            reason="Loose paper match placed on the ground in front of the matchbook.",
        )

    # The open flap (rest pose) stands clear above the back panel and clear of
    # the match comb, so no overlap masking is needed at the displayed pose.

    # ---- cover hero geometry -------------------------------------------
    back_bb = ctx.part_element_world_aabb(cover, elem="back_panel")
    ok_dims = False
    if back_bb is not None:
        (x0, y0, z0), (x1, y1, z1) = back_bb
        ok_dims = (
            abs((x1 - x0) - COVER_W) < 0.003
            and abs((z1 - z0) - BACK_H) < 0.002
            and abs(z0) < 0.001
        )
    ctx.check(
        "back panel matches prompt dimensions and rests on the ground",
        ok_dims,
        details=f"back_panel aabb={back_bb}",
    )

    # Front panel is shorter than the back panel
    front_bb = ctx.part_element_world_aabb(cover, elem="front_panel")
    ctx.check(
        "front panel is shorter than back panel",
        front_bb is not None
        and abs(front_bb[1][2] - front_bb[0][2] - FRONT_H) < 0.002
        and front_bb[1][2] < back_bb[1][2] - 0.005,
        details=f"front_panel aabb={front_bb}",
    )

    # Cover is a folded shell (thin walls), not a solid block
    spine_bb = ctx.part_element_world_aabb(cover, elem="spine")
    ctx.check(
        "cover is a thin-walled folded shell, not a solid block",
        back_bb is not None and front_bb is not None and spine_bb is not None
        and (back_bb[1][1] - back_bb[0][1]) < 0.002
        and (front_bb[1][1] - front_bb[0][1]) < 0.002
        and (spine_bb[1][2] - spine_bb[0][2]) < 0.002,
        details=f"back={back_bb} front={front_bb} spine={spine_bb}",
    )

    # ---- striker strip ------------------------------------------------
    striker_bb = ctx.part_element_world_aabb(cover, elem="striker")
    ctx.check(
        "striker strip is on the outside of the front panel",
        striker_bb is not None
        and striker_bb[1][1] > POCKET_D + T,
        details=f"striker aabb={striker_bb}",
    )

    # ---- match comb ---------------------------------------------------
    ctx.check(
        "ten paper match tabs in the comb",
        all(
            cover.get_visual(f"match_{i}_tab") is not None
            for i in range(N_MATCHES)
        ),
    )
    ctx.check(
        "ten paper match heads in the comb",
        all(
            cover.get_visual(f"match_{i}_head") is not None
            for i in range(N_MATCHES)
        ),
    )

    # Match heads sit at the top of the tabs
    head0_bb = ctx.part_element_world_aabb(cover, elem="match_0_head")
    tab0_bb = ctx.part_element_world_aabb(cover, elem="match_0_tab")
    ctx.check(
        "match heads sit at the top of the tabs",
        head0_bb is not None and tab0_bb is not None
        and head0_bb[1][2] > tab0_bb[1][2] - 0.002,
        details=f"head_0={head0_bb} tab_0={tab0_bb}",
    )

    # Match tabs stand above the front panel (heads visible)
    ctx.check(
        "match tabs extend above the front panel",
        tab0_bb is not None and front_bb is not None
        and tab0_bb[1][2] > front_bb[1][2] + 0.005,
        details=f"tab_0 top={tab0_bb[1][2]} front top={front_bb[1][2]}",
    )

    # ---- flap hinge ---------------------------------------------------
    ctx.check(
        "flap hinge is revolute",
        str(hinge.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis runs along the long (X) axis",
        abs(hinge.axis[0]) > 0.9
        and abs(hinge.axis[1]) < 0.1
        and abs(hinge.axis[2]) < 0.1,
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "hinge origin is at the top fold line of the back panel",
        abs(hinge.origin.xyz[2] - BACK_H) < 0.002
        and abs(hinge.origin.xyz[0]) < 0.002,
        details=f"hinge origin={hinge.origin.xyz}",
    )

    # ---- rest pose: flap leans back, open and clear ------------------------
    flap_rest_bb = ctx.part_world_aabb(flap)
    ctx.check(
        "rest pose shows the flap open (leans back behind the back panel plane)",
        flap_rest_bb is not None
        and back_bb is not None
        and flap_rest_bb[0][1] < back_bb[0][1] - 0.002,
        details=f"flap rest aabb={flap_rest_bb}",
    )
    # The open flap rises above the back panel top, clear of the match comb.
    comb_top = ctx.part_element_world_aabb(cover, elem="match_0_head")
    ctx.check(
        "open flap stands above the back panel top, clear of the matches",
        flap_rest_bb is not None and flap_rest_bb[0][2] > BACK_H - 0.001,
        details=f"flap rest z-min={flap_rest_bb[0][2] if flap_rest_bb else None}, BACK_H={BACK_H}",
    )
    # The open flap does not interpenetrate the match comb (no Y overlap with
    # the comb tabs, which sit in front of the back panel).
    fb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    tb = ctx.part_element_world_aabb(cover, elem="match_0_tab")
    y_clear = fb is not None and tb is not None and fb[1][1] < tb[0][1]
    ctx.check(
        "open flap stays clear of the match comb (no penetration)",
        y_clear,
        details=f"flap_panel={fb} comb_tab={tb}",
    )

    # ---- closed pose: flap folds forward and down over the front -----------
    with ctx.pose({hinge: FLAP_CLOSED_Q}):
        closed_bb = ctx.part_world_aabb(flap)
        ctx.check(
            "closed flap folds forward over the front of the pocket",
            closed_bb is not None
            and closed_bb[1][1] > 0.0
            and closed_bb[1][2] > BACK_H - 0.003,
            details=f"flap closed aabb={closed_bb}",
        )

    # ---- opening further leans the flap further back ----------------------
    with ctx.pose({hinge: FLAP_OPEN_Q}):
        more_open_bb = ctx.part_world_aabb(flap)
        ctx.check(
            "opening further leans the flap further back",
            more_open_bb is not None and flap_rest_bb is not None
            and more_open_bb[0][1] < flap_rest_bb[0][1] - 0.001,
            details=f"more_open={more_open_bb} rest={flap_rest_bb}",
        )

    # ---- loose matches on the ground ----------------------------------
    for i, gm in enumerate(ground_matches):
        gb = ctx.part_world_aabb(gm)
        ctx.check(
            f"ground_match_{i} lies flat on the ground",
            gb is not None and gb[0][2] < 0.001 and gb[1][2] < 0.003,
            details=f"aabb={gb}",
        )
        ctx.expect_gap(
            gm, cover,
            axis="y",
            min_gap=0.003,
            name=f"ground_match_{i} lies clear in front (+Y) of the matchbook",
        )

    ctx.expect_origin_distance(
        ground_matches[0], ground_matches[1],
        axes="xy",
        min_dist=0.005,
        name="loose matches are separated on the ground",
    )

    return ctx.report()


object_model = build_object_model()
