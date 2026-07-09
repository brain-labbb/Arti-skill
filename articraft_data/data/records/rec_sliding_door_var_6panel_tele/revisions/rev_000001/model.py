from __future__ import annotations

# Six-panel telescoping sliding glass door in a slim black aluminium frame.
#
# Object identity and scale:
#   A large storefront / patio telescoping slider, ~4.0 m wide x ~2.4 m tall.
#   One FIXED pane plus five SLIDING panes that telescope and nest along a
#   common horizontal track. The aluminium frame (head, sill, jambs) and the
#   fixed pane form the static root. Each sliding pane rides in its own Y track
#   lane so the leaves can pass one another and stack at the open end.
#
# Coordinates: world up is +Z. The door stands on the ground plane (z=0 upward).
#   X = width (track direction), Y = depth (track lanes), Z = height.
#   The five prismatic joints translate along +X; successive leaves travel
#   farther so they telescope into a single stack while staying engaged.

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
# Overall dimensions (meters)
# ----------------------------------------------------------------------------
FRAME_W = 3.00          # full outer width of the aluminium frame
FRAME_H = 2.40          # full outer height of the frame (sits on ground)
MEMBER = 0.070          # aluminium frame member face width (head/sill/jamb)

# Track lanes in Y. Lane 0 is the rearmost (fixed pane); lanes step forward so
# the leaves can pass one another and telescope into a single stack. The lanes
# are spaced just enough that each slim leaf clears its neighbours in depth,
# keeping the whole frame profile shallow like the photographed slider.
LANE_GAP = 0.040        # center-to-center spacing between adjacent track lanes
LANE0_Y = -2.5 * LANE_GAP   # fixed pane lane (rearmost)
FRAME_D = 5.0 * LANE_GAP + 0.10   # frame depth across all six track lanes

# Opening (clear glazed area inside the frame members)
OPEN_W = FRAME_W - 2.0 * MEMBER     # clear width
OPEN_H = FRAME_H - 2.0 * MEMBER     # clear height
OPEN_X0 = -OPEN_W / 2.0             # left edge of the clear opening
OPEN_Z0 = MEMBER                    # bottom of the clear opening (above sill)
OPEN_ZC = OPEN_Z0 + OPEN_H / 2.0    # vertical center of the glazed area

# Track channels (shallow aluminium guides at the head and sill of the
# opening). The leaf rails engage the channels (captured top and bottom) so
# each leaf is physically carried by the frame, like a real sliding door.
TRACK_T = 0.022         # track channel height in Z
TRACK_ENGAGE = 0.007    # how far each leaf rail enters the track channel

# Each of the six panes spans one sixth of the clear opening width, with a
# small overlap at the meeting stiles so closed leaves interlock (no gaps).
# The overlap is carried on the LEADING (-X, stacking) side of each leaf so the
# trailing edge of the outermost leaf stays flush inside the frame jamb.
PANE_PITCH = OPEN_W / 6.0           # nominal slot width / center spacing
OVERLAP = 0.030                     # meeting-stile interlock width
PANE_W = PANE_PITCH + OVERLAP       # leaf width incl. meeting-stile overlap
# Leaf height: the rails reach into the head and sill channels by TRACK_ENGAGE
# so the leaf is captured top and bottom (a small intentional capture overlap).
PANE_H = OPEN_H - 2.0 * (TRACK_T - TRACK_ENGAGE)

# Leaf construction. Slim members keep the thin black mullion / clear-glass
# look of the reference photo.
STILE = 0.040           # vertical edge member width of each leaf
RAIL = 0.048            # top/bottom rail height of each leaf
LEAF_T = 0.020          # leaf aluminium frame thickness (in Y)
GLASS_T = 0.006         # glazing thickness

NEST = 0.040            # X engagement kept between stacked leaves when open


def _lane_y(index: int) -> float:
    """Y center of track lane `index` (0 = fixed/rear, 5 = frontmost)."""
    return LANE0_Y + index * LANE_GAP


def _pane_center_x_closed(slot: int) -> float:
    """X center of the pane that fills opening slot `slot` (0..5) when closed.

    Slots tile the opening left-to-right. The leaf is slightly wider than its
    slot; the extra width is carried on the leading (-X) side, so the leaf's
    trailing (+X) edge stays at the slot boundary and the outermost leaf is
    flush inside the frame jamb.
    """
    return OPEN_X0 + (slot + 0.5) * PANE_PITCH - OVERLAP / 2.0


def _add_leaf(part, *, glass_mat, alu_mat) -> None:
    """Author one glazed leaf centered on its own part frame at the origin.

    The leaf is a slim aluminium border (two stiles, top and bottom rail)
    around a single transparent glass sheet, plus a recessed pull handle.
    Part frame origin sits at the leaf center; geometry is symmetric about it.
    """
    half_w = PANE_W / 2.0
    half_h = PANE_H / 2.0

    # Glass sheet (transparent), inset behind the aluminium border.
    part.visual(
        Box((PANE_W - 2.0 * STILE + 0.004, GLASS_T, PANE_H - 2.0 * RAIL + 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass_mat,
        name="glass",
    )
    # Top and bottom rails run the full leaf width and are the members that
    # engage the head and sill track channels (so they capture the leaf).
    part.visual(
        Box((PANE_W, LEAF_T, RAIL)),
        origin=Origin(xyz=(0.0, 0.0, half_h - RAIL / 2.0)),
        material=alu_mat,
        name="rail_top",
    )
    part.visual(
        Box((PANE_W, LEAF_T, RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -(half_h - RAIL / 2.0))),
        material=alu_mat,
        name="rail_bottom",
    )
    # Vertical stiles run only between the rails so they stay clear of the
    # tracks (only the rails enter the channels).
    stile_h = PANE_H - 2.0 * RAIL
    part.visual(
        Box((STILE, LEAF_T, stile_h)),
        origin=Origin(xyz=(-(half_w - STILE / 2.0), 0.0, 0.0)),
        material=alu_mat,
        name="stile_0",
    )
    part.visual(
        Box((STILE, LEAF_T, stile_h)),
        origin=Origin(xyz=(half_w - STILE / 2.0, 0.0, 0.0)),
        material=alu_mat,
        name="stile_1",
    )
    # Vertical pull handle on the leading (left) stile, centered at hand height.
    # Kept thin and shallow so it clears the leaf in the adjacent forward lane.
    part.visual(
        Box((0.018, 0.012, 0.300)),
        origin=Origin(xyz=(-(half_w - STILE - 0.004), LEAF_T / 2.0 + 0.006, 0.0)),
        material=alu_mat,
        name="handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_sliding_glass_door")

    # Restrained real-world materials.
    alu = model.material("black_aluminium", rgba=(0.07, 0.075, 0.08, 1.0))
    alu_dark = model.material("anodized_black", rgba=(0.05, 0.052, 0.055, 1.0))
    glass = model.material("clear_glass", rgba=(0.62, 0.70, 0.72, 0.22))

    # ---------------------------------------------------------------- frame
    # Static root: outer aluminium frame (head, sill, two jambs) + the fixed
    # pane built directly into the frame at the rear track lane.
    frame = model.part("frame")

    half_w = FRAME_W / 2.0
    yc = 0.0  # frame box centered in Y; lanes are offset within FRAME_D

    # Sill (bottom member) sitting on the ground.
    frame.visual(
        Box((FRAME_W, FRAME_D, MEMBER)),
        origin=Origin(xyz=(0.0, yc, MEMBER / 2.0)),
        material=alu,
        name="sill",
    )
    # Head (top member).
    frame.visual(
        Box((FRAME_W, FRAME_D, MEMBER)),
        origin=Origin(xyz=(0.0, yc, FRAME_H - MEMBER / 2.0)),
        material=alu,
        name="head",
    )
    # Jambs (left/right vertical members), between sill and head.
    jamb_h = FRAME_H - 2.0 * MEMBER
    frame.visual(
        Box((MEMBER, FRAME_D, jamb_h)),
        origin=Origin(xyz=(-(half_w - MEMBER / 2.0), yc, MEMBER + jamb_h / 2.0)),
        material=alu,
        name="jamb_0",
    )
    frame.visual(
        Box((MEMBER, FRAME_D, jamb_h)),
        origin=Origin(xyz=(half_w - MEMBER / 2.0, yc, MEMBER + jamb_h / 2.0)),
        material=alu,
        name="jamb_1",
    )

    # Track guides: shallow top and bottom aluminium channels running across the
    # opening that the sliding leaves ride between. Multi-lane in Y so each leaf
    # has its own running surface. The leaves clear them by TRACK_CLR in Z.
    frame.visual(
        Box((OPEN_W, FRAME_D - 0.02, TRACK_T)),
        origin=Origin(xyz=(0.0, yc, OPEN_Z0 + TRACK_T / 2.0)),
        material=alu_dark,
        name="bottom_track",
    )
    frame.visual(
        Box((OPEN_W, FRAME_D - 0.02, TRACK_T)),
        origin=Origin(xyz=(0.0, yc, OPEN_Z0 + OPEN_H - TRACK_T / 2.0)),
        material=alu_dark,
        name="top_track",
    )

    # Fixed pane built into the frame, occupying opening slot 0 in the rear lane.
    fixed_y = _lane_y(0)
    fixed_cx = _pane_center_x_closed(0)
    fhw = PANE_W / 2.0
    fhh = PANE_H / 2.0
    frame.visual(
        Box((PANE_W - 2.0 * STILE + 0.004, GLASS_T, PANE_H - 2.0 * RAIL + 0.004)),
        origin=Origin(xyz=(fixed_cx, fixed_y, OPEN_ZC)),
        material=glass,
        name="fixed_glass",
    )
    frame.visual(
        Box((STILE, LEAF_T, PANE_H)),
        origin=Origin(xyz=(fixed_cx - (fhw - STILE / 2.0), fixed_y, OPEN_ZC)),
        material=alu,
        name="fixed_stile_0",
    )
    frame.visual(
        Box((STILE, LEAF_T, PANE_H)),
        origin=Origin(xyz=(fixed_cx + (fhw - STILE / 2.0), fixed_y, OPEN_ZC)),
        material=alu,
        name="fixed_stile_1",
    )
    f_rail_len = PANE_W - 2.0 * STILE
    frame.visual(
        Box((f_rail_len, LEAF_T, RAIL)),
        origin=Origin(xyz=(fixed_cx, fixed_y, OPEN_ZC + (fhh - RAIL / 2.0))),
        material=alu,
        name="fixed_rail_top",
    )
    frame.visual(
        Box((f_rail_len, LEAF_T, RAIL)),
        origin=Origin(xyz=(fixed_cx, fixed_y, OPEN_ZC - (fhh - RAIL / 2.0))),
        material=alu,
        name="fixed_rail_bottom",
    )

    # ------------------------------------------------------------- sliding panes
    # Five sliding leaves, each in its own forward track lane (1..5). When
    # closed each fills the next opening slot. Positive prismatic q slides them
    # along +X to stack/telescope toward the fixed side.
    panes = []
    for i in range(5):
        slot = i + 1          # opening slot filled when closed (1..5)
        leaf = model.part(f"pane_{i}")
        _add_leaf(leaf, glass_mat=glass, alu_mat=alu)
        panes.append((leaf, slot))

    # Joint origins: place each leaf's joint frame at its CLOSED center so that
    # at q=0 the leaf seats in its slot. The leaf part frame (origin) coincides
    # with the joint frame at q=0, so the leaf appears centered there.
    #
    # Telescoping travel: when open, all sliding leaves stack over the fixed
    # pane (slot 0). The inner leaf (slot 1) travels one pitch; the next travels
    # two; etc. so leaf i (slot i+1) travels (i+1) pitches minus a small nesting
    # overlap, keeping successive leaves engaged.
    for i, (leaf, slot) in enumerate(panes):
        lane_y = _lane_y(slot)
        cx = _pane_center_x_closed(slot)
        # Travel toward -X (toward the fixed pane at slot 0). Each successive
        # leaf travels one extra pitch so they telescope and nest.
        travel = slot * PANE_PITCH - i * NEST
        model.articulation(
            f"pane_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=leaf,
            origin=Origin(xyz=(cx, lane_y, OPEN_ZC)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=400.0, velocity=0.6, lower=0.0, upper=travel
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panes = [object_model.get_part(f"pane_{i}") for i in range(5)]
    slides = [object_model.get_articulation(f"pane_{i}_slide") for i in range(5)]

    # --- Frame is the single static root; sliding leaves hang off it. -------
    roots = object_model.root_parts()
    ctx.check(
        "frame is the sole root",
        len(roots) == 1 and roots[0].name == "frame",
        details=f"roots={[p.name for p in roots]}",
    )

    # --- Multiplicity: exactly five sliding panes plus one fixed (six total). --
    ctx.check(
        "five sliding panes exist",
        len(panes) == 5,
        details=f"expected 5 sliding panes, got {len(panes)}",
    )
    ctx.check(
        "five prismatic slide joints exist",
        len(slides) == 5,
        details=f"expected 5 prismatic joints, got {len(slides)}",
    )

    # --- Hero: transparent glazing is present on the fixed pane and leaves. --
    fixed_glass = frame.get_visual("fixed_glass")
    ctx.check(
        "fixed glass exists",
        fixed_glass is not None,
        details="frame must include the built-in fixed glass pane",
    )
    for i, leaf in enumerate(panes):
        g = leaf.get_visual("glass")
        ctx.check(
            f"pane_{i} has glazing",
            g is not None,
            details=f"pane_{i} must carry a transparent glass sheet",
        )

    # --- Closed pose: every leaf covers (most of) the opening height. -------
    for i, leaf in enumerate(panes):
        ctx.expect_overlap(
            leaf,
            frame,
            axes="z",
            min_overlap=PANE_H - 0.01,
            name=f"pane_{i} spans the opening height when closed",
        )

    # --- Each leaf is captured top and bottom by the head and sill tracks. ---
    # The rails reach TRACK_ENGAGE into the channels; prove they touch/engage
    # the frame so no leaf reads as a floating part, and allow that small,
    # intentional capture overlap scoped to the rail/track elements.
    for i, leaf in enumerate(panes):
        ctx.allow_overlap(
            leaf,
            frame,
            elem_a="rail_top",
            elem_b="top_track",
            reason=(
                "The leaf top rail intentionally enters the head track channel "
                "to capture and carry the sliding leaf, like a real door."
            ),
        )
        ctx.allow_overlap(
            leaf,
            frame,
            elem_a="rail_bottom",
            elem_b="bottom_track",
            reason=(
                "The leaf bottom rail intentionally enters the sill track "
                "channel so the leaf rides captured in the frame."
            ),
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="rail_top",
            elem_b="top_track",
            name=f"pane_{i} top rail engages the head track",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="rail_bottom",
            elem_b="bottom_track",
            name=f"pane_{i} bottom rail engages the sill track",
        )

    # --- Closed pose: leaves stay engaged with their neighbor (no X gaps). ---
    # Adjacent closed leaves (and leaf 0 with the fixed pane) overlap in X at
    # the meeting stiles so the closed door has no daylight gaps.
    ctx.expect_overlap(
        panes[0],
        frame,
        axes="x",
        min_overlap=0.010,
        elem_b="fixed_stile_1",
        name="inner leaf interlocks with the fixed pane when closed",
    )
    for i in range(1, 5):
        ctx.expect_overlap(
            panes[i],
            panes[i - 1],
            axes="x",
            min_overlap=0.010,
            name=f"pane_{i} interlocks with pane_{i - 1} when closed",
        )

    # --- Leaves ride in distinct forward track lanes (separated in Y). ------
    # Each lane is offset so the leaves can pass one another while telescoping.
    # Measured on the glazing planes (the pull handles intentionally reach part
    # way into the next lane, so the whole-part gap is smaller).
    for i in range(4):
        ctx.expect_gap(
            panes[i + 1],
            panes[i],
            axis="y",
            positive_elem="glass",
            negative_elem="glass",
            min_gap=LANE_GAP - GLASS_T - 0.002,
            name=f"pane_{i + 1} rides forward of pane_{i} (separate lane)",
        )

    # --- Each prismatic joint actually slides its leaf along the track. -----
    rest_x = [ctx.part_world_position(p)[0] for p in panes]
    for i, (leaf, joint) in enumerate(zip(panes, slides)):
        upper = joint.motion_limits.upper
        with ctx.pose({joint: upper}):
            open_x = ctx.part_world_position(leaf)[0]
        ctx.check(
            f"pane_{i} slides toward the stack (-X) when opened",
            open_x < rest_x[i] - 0.20,
            details=f"rest_x={rest_x[i]:.3f}, open_x={open_x:.3f}",
        )

    # --- Telescoped pose: all leaves stack and remain engaged (nested). -----
    open_pose = {j: j.motion_limits.upper for j in slides}
    with ctx.pose(open_pose):
        # Successive leaves stay engaged with the previous one along X.
        for i in range(1, 5):
            ctx.expect_overlap(
                panes[i],
                panes[i - 1],
                axes="x",
                min_overlap=NEST - 0.005,
                name=f"pane_{i} stays nested with pane_{i - 1} when fully open",
            )
        # Leaves remain within the frame footprint (do not exit the opening).
        for i, leaf in enumerate(panes):
            ctx.expect_within(
                leaf,
                frame,
                axes="x",
                margin=0.06,
                name=f"pane_{i} stays within the frame when stacked",
            )

    return ctx.report()


object_model = build_object_model()
