from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    tube_from_spline_points,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Reflective glass entry assembly (Z-up: stands upright on the ground plane).
#
# Two mirror-like glass leaves side by side inside one slim dark metal frame.
# BOTH leaves are operable: each swings open on a vertical hinge along its
# outer edge, meeting at the fixed center mullion (a double / French door).
# Each glass leaf carries a wavy S-curved brass pull handle on standoffs.
#
# Coordinate convention (world UP = +Z):
#   - height runs along +Z, sill/base at z=0, top ~2.05 m.
#   - width runs along X.
#   - glass thickness + swing-normal run along Y.
#   - hinge axis is vertical, (0, 0, 1); both leaves swing outward (-Y).
# ---------------------------------------------------------------------------

# Real-world dimensions (meters).
LEAF_W = 0.90          # width of one glass leaf/panel (along X)
LEAF_H = 2.05          # height of glass leaf (along Z)
GLASS_T = 0.012        # glass thickness (along Y, the door normal)
FRAME_W = 0.045        # frame member visible face width
FRAME_D = 0.060        # frame member depth (along Y, the door normal)
SILL_H = 0.030         # bottom sill height

# Overall opening: two units side by side, with jambs between/around them.
UNIT_W = LEAF_W
GAP = 0.006            # reveal gap between leaf and jamb

# X layout: side panel on -X side, swinging leaf on +X side.
# Frame center at x=0. Center mullion between the two units sits at x=0.
PANEL_CX = -(UNIT_W / 2.0 + FRAME_W / 2.0 + GAP)
LEAF_CX = +(UNIT_W / 2.0 + FRAME_W / 2.0 + GAP)

# Y (door normal): glass centered at y=0; frame straddles it.
HANDLE_STANDOFF = 0.040
HANDLE_TUBE_R = 0.011

# Vertical extents (along Z). Sill base sits on the ground plane at z=0.
OPENING_H = LEAF_H + 2.0 * FRAME_W
HEAD_CZ = OPENING_H - FRAME_W / 2.0      # head member center z
SILL_CZ = SILL_H / 2.0                   # sill member center z (base at z=0)
GLASS_CZ = (HEAD_CZ - FRAME_W / 2.0 + SILL_CZ + SILL_H / 2.0) / 2.0  # vertical center of glass span

# Outer jamb x positions (outside edges of both units).
LEFT_JAMB_X = PANEL_CX - UNIT_W / 2.0 - GAP - FRAME_W / 2.0
RIGHT_JAMB_X = LEAF_CX + UNIT_W / 2.0 + GAP + FRAME_W / 2.0
MULLION_X = 0.0  # central jamb between the two units

FRAME_SPAN_X = RIGHT_JAMB_X - LEFT_JAMB_X + FRAME_W


def _s_handle_points(height: float, bow: float, depth: float) -> list[tuple[float, float, float]]:
    """Wavy S-curved vertical pull handle path in local frame.

    The handle runs vertically along Z. X stays ~0 (mounting plane), Y bows
    outward in a smooth S so it reads as an ornate wavy pull. The two ends bend
    back toward the glass so standoffs can attach.
    """
    n = 9
    pts: list[tuple[float, float, float]] = []
    for i in range(n):
        t = i / (n - 1)
        z = -height / 2.0 + t * height
        # Two-lobe S wave; Y bows outward toward the room.
        wave = math.sin(t * math.pi * 2.0)
        y = depth + bow * wave
        if i == 0 or i == n - 1:
            # Pull the very ends back toward the mounting plane (the standoffs).
            y = depth * 0.35
        pts.append((0.0, y, z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reflective_glass_door")

    dark_frame = model.material("dark_frame", color=(0.12, 0.12, 0.14))
    glass = model.material("glass", color=(0.62, 0.70, 0.74, 0.45))
    brass = model.material("brass", color=(0.76, 0.58, 0.22))

    # -------------------------------------------------------------------
    # FIXED frame / casing (root): head, sill, and three vertical jambs.
    # -------------------------------------------------------------------
    frame = model.part("frame")

    # Head and sill span the full opening width.
    frame.visual(
        Box((FRAME_SPAN_X, FRAME_D, FRAME_W)),
        origin=Origin(xyz=(0.0, 0.0, HEAD_CZ)),
        material=dark_frame,
        name="head_jamb",
    )
    frame.visual(
        Box((FRAME_SPAN_X, FRAME_D, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, SILL_CZ)),
        material=dark_frame,
        name="sill",
    )
    jamb_h = (HEAD_CZ - FRAME_W / 2.0) - (SILL_CZ + SILL_H / 2.0)
    jamb_cz = ((HEAD_CZ - FRAME_W / 2.0) + (SILL_CZ + SILL_H / 2.0)) / 2.0
    for name, jx in (("left_jamb", LEFT_JAMB_X), ("center_mullion", MULLION_X), ("right_jamb", RIGHT_JAMB_X)):
        frame.visual(
            Box((FRAME_W, FRAME_D, jamb_h)),
            origin=Origin(xyz=(jx, 0.0, jamb_cz)),
            material=dark_frame,
            name=name,
        )

    # -------------------------------------------------------------------
    # Two operable glass leaves (both swing open), built from ONE parametric
    # leaf instantiated mirror-symmetrically about X. Each leaf hinges on its
    # outer edge against the matching outer jamb; their free edges meet at the
    # fixed center mullion.
    # -------------------------------------------------------------------
    _build_leaf(model, frame, +1, jamb_h, jamb_cz, dark_frame, glass, brass)
    _build_leaf(model, frame, -1, jamb_h, jamb_cz, dark_frame, glass, brass)

    return model


def _build_leaf(model, frame, s, jamb_h, jamb_cz, dark_frame, glass, brass):
    """Build one operable glass leaf on side ``s`` (+1 right, -1 left).

    The leaf is hinged at its OUTER edge and extends toward the center. The
    two calls (s=+1, s=-1) produce mirror-symmetric leaves about the X axis.
    """
    side = "right" if s > 0 else "left"
    leaf = model.part(f"{side}_leaf")

    # Outer edge of this unit = hinge line. In leaf-local coords the hinge is at
    # x=0 and the glass extends toward the center (the -s direction in world).
    unit_cx = s * (UNIT_W / 2.0 + FRAME_W / 2.0 + GAP)  # LEAF_CX / PANEL_CX
    hinge_x = unit_cx + s * (UNIT_W / 2.0)              # outer edge (world x)
    glass_local_cx = -s * (UNIT_W / 2.0)               # glass center (local x)

    leaf.visual(
        Box((UNIT_W, GLASS_T, LEAF_H)),
        origin=Origin(xyz=(glass_local_cx, 0.0, GLASS_CZ)),
        material=glass,
        name=f"{side}_leaf_glass",
    )
    _add_glass_trim(leaf, glass_local_cx, dark_frame, prefix=f"{side}_leaf")
    # Handle near the free (center-facing) edge, like a real pull on the leading
    # edge. Mirror the right-leaf offset of -UNIT_W/2 + 0.10 across X.
    handle_x = glass_local_cx + s * 0.10
    _add_brass_handle(leaf, glass_local_cx, brass, prefix=f"{side}_leaf", handle_x_offset=handle_x)

    # Slim dark hinge stile on the leaf's hinge (outer) edge. It bridges the
    # reveal gap so the leaf butts the outer jamb (no floating leaf), reaching
    # just to the jamb inner face with a 1 mm seat.
    outer_jamb_x = s * RIGHT_JAMB_X                     # RIGHT_JAMB_X / LEFT_JAMB_X
    jamb_inner_x = outer_jamb_x - s * (FRAME_W / 2.0)
    stile_reach = (jamb_inner_x - hinge_x) + s * 0.001
    stile_w = FRAME_W
    leaf.visual(
        Box((stile_w, FRAME_D * 0.7, jamb_h)),
        origin=Origin(xyz=(stile_reach - s * (stile_w / 2.0), 0.0, jamb_cz)),
        material=dark_frame,
        name=f"{side}_leaf_hinge_stile",
    )

    # Two hinge knuckles (barrels) on the leaf hinge line, nestled at the gap
    # between the leaf stile and the jamb (real hardware seats into the jamb).
    knuckle_x = jamb_inner_x - hinge_x  # local x at the jamb inner face
    for sgn, tag in ((1, "upper"), (-1, "lower")):
        knuckle_z = jamb_cz + sgn * (jamb_h / 2.0 - 0.20)
        leaf.visual(
            Cylinder(radius=0.011, length=0.060),
            origin=Origin(xyz=(knuckle_x, 0.0, knuckle_z)),
            material=dark_frame,
            name=f"{side}_leaf_hinge_knuckle_{tag}",
        )

    # Hinge: vertical axis at the leaf outer edge. The leaf extends toward the
    # center (-s in world); both leaves swing OUTWARD (toward -Y) as the door
    # opens. For the right leaf (s=+1) that is positive q; for the left leaf
    # (s=-1) it is negative q. Limits are mirrored accordingly.
    if s > 0:
        limits = MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=1.6)
    else:
        limits = MotionLimits(effort=30.0, velocity=2.0, lower=-1.6, upper=0.0)
    model.articulation(
        f"frame_to_{side}_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )

    return leaf


def _add_glass_trim(part, glass_cx: float, material, *, prefix: str) -> None:
    """Thin dark retaining lip framing one glass unit (top/bottom/sides)."""
    lip_w = 0.018
    lip_t = GLASS_T + 0.004
    half_w = UNIT_W / 2.0
    half_h = LEAF_H / 2.0
    cz = GLASS_CZ
    # Top and bottom rails (run along X at the top/bottom of the glass span).
    for sgn, tag in ((1, "top"), (-1, "bottom")):
        part.visual(
            Box((UNIT_W - 2.0 * lip_w, lip_t, lip_w)),
            origin=Origin(xyz=(glass_cx, 0.0, cz + sgn * (half_h - lip_w / 2.0))),
            material=material,
            name=f"{prefix}_glass_rail_{tag}",
        )
    # Left and right stiles (run along Z at the left/right edges of the glass).
    for sgn, tag in ((1, "right"), (-1, "left")):
        part.visual(
            Box((lip_w, lip_t, LEAF_H)),
            origin=Origin(xyz=(glass_cx + sgn * (half_w - lip_w / 2.0), 0.0, cz)),
            material=material,
            name=f"{prefix}_glass_stile_{tag}",
        )


def _add_brass_handle(part, glass_cx: float, material, *, prefix: str, handle_x_offset: float | None = None) -> None:
    """Wavy S-curved brass pull handle on the +Y face, on two standoffs."""
    hx = glass_cx if handle_x_offset is None else handle_x_offset
    cz = GLASS_CZ
    handle_h = 1.05
    bow = 0.022

    pts = _s_handle_points(handle_h, bow, HANDLE_STANDOFF)
    handle_mesh = tube_from_spline_points(
        pts,
        radius=HANDLE_TUBE_R,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    # Position handle bar at hx, on +Y side of glass, centered vertically at cz.
    part.visual(
        mesh_from_geometry(handle_mesh, f"{prefix}_handle_bar"),
        origin=Origin(xyz=(hx, GLASS_T / 2.0, cz)),
        material=material,
        name=f"{prefix}_handle_bar",
    )
    # Two cylindrical standoffs connecting the bar ends to the glass (along +Y).
    standoff_len = HANDLE_STANDOFF * 0.35
    for sgn, tag in ((1, "top"), (-1, "bottom")):
        end_z = cz + sgn * handle_h / 2.0
        part.visual(
            Cylinder(radius=0.008, length=standoff_len),
            # Cylinder is built along +Z locally; rotate so it points along +Y
            # (the door normal), bridging the glass face and the handle end.
            origin=Origin(
                xyz=(hx, GLASS_T / 2.0 + standoff_len / 2.0, end_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_standoff_{tag}",
        )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    # Both leaves are operable and mirror-symmetric. Iterate over each side.
    leaves = []
    for side, s, outer_jamb in (("right", +1, "right_jamb"), ("left", -1, "left_jamb")):
        leaf = object_model.get_part(f"{side}_leaf")
        hinge = object_model.get_articulation(f"frame_to_{side}_leaf")
        leaves.append((side, s, leaf, hinge, outer_jamb))

    # The hinge knuckles are real barrel hardware that seats across the gap and
    # laps the jamb edge (a genuine seated-hardware overlap). Scope an allowance
    # for the exact knuckle/jamb element pairs and prove the lap with checks.
    for side, _s, leaf, _hinge, outer_jamb in leaves:
        jamb_v = frame.get_visual(outer_jamb)
        for tag in ("upper", "lower"):
            knuckle = leaf.get_visual(f"{side}_leaf_hinge_knuckle_{tag}")
            ctx.allow_overlap(
                leaf,
                frame,
                elem_a=knuckle,
                elem_b=jamb_v,
                reason="hinge barrel knuckle laps the jamb edge (real seated hinge hardware)",
            )
            ctx.expect_overlap(
                leaf,
                frame,
                elem_a=knuckle,
                elem_b=jamb_v,
                axes="x",
                name=f"{side}_hinge_knuckle_{tag}_laps_jamb",
            )

    # Orientation: the assembly stands upright, height along +Z, base on z=0.
    boxes = [ctx.part_world_aabb(frame)]
    boxes += [ctx.part_world_aabb(leaf) for _, _, leaf, _, _ in leaves]
    boxes = [b for b in boxes if b is not None]
    if boxes:
        ox0 = min(b[0][0] for b in boxes)
        oy0 = min(b[0][1] for b in boxes)
        oz0 = min(b[0][2] for b in boxes)
        ox1 = max(b[1][0] for b in boxes)
        oy1 = max(b[1][1] for b in boxes)
        oz1 = max(b[1][2] for b in boxes)
        dx, dy, dz = ox1 - ox0, oy1 - oy0, oz1 - oz0
        ctx.check("rests_on_ground", abs(oz0) < 0.01, details=f"zmin={oz0}")
        ctx.check("height_along_z", abs(dz - OPENING_H) < 0.05, details=f"dz={dz}")
        ctx.check("upright_tallest_axis_z", dz > dx and dz > dy, details=f"dx={dx},dy={dy},dz={dz}")

    for side, _s, _leaf, hinge, _ in leaves:
        ctx.check(
            f"{side}_hinge_axis_vertical",
            abs(hinge.axis[2]) > 0.99 and abs(hinge.axis[0]) < 0.01 and abs(hinge.axis[1]) < 0.01,
            details=f"axis={hinge.axis}",
        )

    # Hero feature: both glass leaves present, sized realistically, with handles
    # mounted on standoffs, and each leaf swings open about its outer-edge hinge.
    for side, s, leaf, hinge, outer_jamb in leaves:
        leaf_glass = leaf.get_visual(f"{side}_leaf_glass")
        ctx.check(f"{side}_leaf_glass_present", leaf_glass is not None)

        lg_aabb = ctx.part_element_world_aabb(leaf, elem=leaf_glass)
        if lg_aabb is not None:
            (lx0, _, lz0), (lx1, _, lz1) = lg_aabb
            ctx.check(
                f"{side}_leaf_glass_width", abs((lx1 - lx0) - UNIT_W) < 0.02, details=f"w={lx1 - lx0}"
            )
            ctx.check(
                f"{side}_leaf_glass_height", abs((lz1 - lz0) - LEAF_H) < 0.02, details=f"h={lz1 - lz0}"
            )

        ctx.check(f"{side}_leaf_handle_present", leaf.get_visual(f"{side}_leaf_handle_bar") is not None)

        # Handle is mounted (standoffs contact the glass face, not floating).
        ctx.expect_contact(
            leaf,
            leaf,
            elem_a=leaf.get_visual(f"{side}_leaf_standoff_top"),
            elem_b=leaf_glass,
            name=f"{side}_leaf_handle_standoff_on_glass",
        )

        jamb_v = frame.get_visual(outer_jamb)

        def _glass_center_y(_leaf=leaf, _glass=leaf_glass) -> float | None:
            ab = ctx.part_element_world_aabb(_leaf, elem=_glass)
            if ab is None:
                return None
            (_, y0, _), (_, y1, _) = ab
            return (y0 + y1) / 2.0

        # Closed: hinge edge meets the outer jamb (connected, no floating leaf).
        with ctx.pose({hinge: 0.0}):
            ctx.expect_contact(
                leaf,
                frame,
                elem_a=leaf_glass,
                elem_b=jamb_v,
                contact_tol=0.05,
                name=f"{side}_leaf_hinge_edge_near_jamb_closed",
            )
            closed_y = _glass_center_y()

        # Open: leaf swings clear toward -Y while the hinge edge stays put.
        with ctx.pose({hinge: s * 1.4}):
            ctx.expect_contact(
                leaf,
                frame,
                elem_a=leaf_glass,
                elem_b=jamb_v,
                contact_tol=0.08,
                name=f"{side}_leaf_hinge_edge_stays_connected_open",
            )
            open_y = _glass_center_y()

        ctx.check(
            f"{side}_leaf_swings_open",
            closed_y is not None and open_y is not None and abs(open_y - closed_y) > 0.1,
            details=f"closed_y={closed_y}, open_y={open_y}",
        )

    return ctx.report()


object_model = build_object_model()
