from __future__ import annotations

# Ornate double door in honey-brown wood with a dark steel surround.
# Two leaves whose raised moldings form a large central circle with dark
# recessed insets, flanked by narrow fixed sidelight panels and topped by a
# rectangular transom panel, with central pull handles.
#
# Coordinate frame: X = width (right +), Y = depth/thickness (toward viewer +),
# Z = height (up). The fixed dark-steel surround (with sidelights + transom) is
# the root. Two operable wood leaves meet at the center and swing on vertical
# (Z) hinge axes at their OUTER edges.
#
# Symmetry contract: the two leaves are MIRROR images across the world X=0
# plane. Each leaf is authored in its own local frame with the hinge edge at
# local x=0 and the body extending toward `sign * X` (sign=-1 for door_0,
# sign=+1 for door_1). All decoration (the half-ring of the central circle,
# the dark recessed inset, the panel border, and the pull handle) is mirrored
# in X via `sign` while the FRONT FACE of BOTH leaves stays at +Y. door_0 is
# placed at the left jamb, door_1 at the right jamb; the right leaf opens about
# -Z so both leaves swing outward (toward +Y) symmetrically. Because both front
# faces point +Y and the circle is centered on the shared meeting edge, the two
# molding half-rings complete one circle and all grain/panels line up.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
LEAF_W = 0.82          # width of one door leaf
LEAF_H = 2.30          # height of leaf (matches transom underside)
LEAF_T = 0.045         # leaf thickness
OPENING_W = 2.0 * LEAF_W  # 1.64 m double-door opening

FRAME_DEPTH = 0.18     # surround depth (front-to-back)
JAMB_W = 0.10          # width of the dark steel jamb members
SIDELIGHT_W = 0.22     # width of each narrow sidelight panel
TRANSOM_H = 0.34       # height of the transom band above the leaves
HEAD_RAIL_H = 0.07     # mullion/rail between leaves and transom

CENTER_REVEAL = 0.006  # half-gap at the central meeting edge (per leaf)

# Overall surround spans the leaves + two sidelights + jambs.
SURROUND_W = OPENING_W + 2.0 * (SIDELIGHT_W + JAMB_W) + 2.0 * JAMB_W
SURROUND_H = LEAF_H + HEAD_RAIL_H + TRANSOM_H + 2.0 * JAMB_W

# X position of the inner edges of each sidelight opening (= door opening edge)
DOOR_HALF = OPENING_W / 2.0

# Central circular motif: centered at the meeting edge so both leaves' halves
# combine into one circle spanning the pair.
RING_OUTER_R = 0.60
RING_INNER_R = 0.50

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
WOOD = Material(name="honey_wood", rgba=(0.78, 0.47, 0.22, 1.0))
WOOD_DARK = Material(name="wood_shadow", rgba=(0.58, 0.33, 0.15, 1.0))
STEEL = Material(name="dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
INSET_DARK = Material(name="recess_dark", rgba=(0.10, 0.11, 0.12, 1.0))
HANDLE = Material(name="bronze_handle", rgba=(0.28, 0.24, 0.20, 1.0))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
# Each leaf is authored in a local frame with the hinge edge at local x=0 and
# the body extending toward `sign * X`. The meeting (inner) edge is therefore
# at local x = sign * LEAF_W, where the central circle is centered. The front
# face is +Y for BOTH leaves (sign only mirrors X), guaranteeing that the two
# decorated faces match and the half-rings complete one circle.

def _leaf_body(sign: float) -> cq.Workplane:
    """One door leaf body with stile/rail border and a raised half-ring of the
    central circle. `sign` mirrors the geometry across X within the local frame.
    """
    half_t = LEAF_T / 2.0

    # The hinge edge is at local x=0. The nominal meeting edge (where the circle
    # is centered) is at x_meet = sign*LEAF_W. The physical body is trimmed by
    # CENTER_REVEAL on the meeting side so the two closed leaves leave a small
    # central reveal and do not interpenetrate, while the circle stays centered
    # on the nominal meeting edge so the two half-rings still complete one circle.
    body_w = LEAF_W - CENTER_REVEAL
    x_meet = sign * LEAF_W                 # nominal circle center (meeting edge)
    x_body_edge = sign * body_w            # trimmed physical meeting edge
    x_lo = min(0.0, x_body_edge)
    x_hi = max(0.0, x_body_edge)
    x_mid = (x_lo + x_hi) / 2.0

    # Base slab (centered in Y and Z, spanning the trimmed leaf width in X).
    leaf = (
        cq.Workplane("XY")
        .box(body_w, LEAF_T, LEAF_H, centered=(True, True, True))
        .translate((x_mid, 0.0, 0.0))
    )

    # Recessed flat panel field: cut a shallow rectangular pocket on the front
    # face, leaving stiles/rails as a raised border.
    border = 0.11
    pocket_w = body_w - 2.0 * border
    pocket_h = LEAF_H - 2.0 * border
    pocket_depth = 0.012
    pocket = (
        cq.Workplane("XY")
        .box(pocket_w, pocket_depth * 2, pocket_h, centered=(True, True, True))
        .translate((x_mid, half_t - pocket_depth + 0.0001, 0.0))
    )
    leaf = leaf.cut(pocket)

    # Raised molding "circle": concentric raised rings on the front face,
    # centered exactly at the meeting edge (local x = x_meet) so the two leaves
    # together read as ONE large circle spanning the pair. The half of each ring
    # that would spill past the leaf edge is trimmed by intersecting with the
    # leaf footprint.
    cx = x_meet
    cz = 0.0
    raise_h = 0.020
    front_y = half_t

    # Footprint clip: a slab matching the (trimmed) leaf bounds, deep enough to
    # keep all raised geometry that lies over the leaf face.
    clip = (
        cq.Workplane("XY")
        .box(body_w, LEAF_T + 2 * raise_h, LEAF_H, centered=(True, True, True))
        .translate((x_mid, 0.0, 0.0))
    )

    # The XZ workplane normal points -Y, so extruding a negative amount pushes
    # the raised ring toward +Y (proud of the front face).
    ring = (
        cq.Workplane("XZ", origin=(cx, front_y, cz))
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(-raise_h)
    )
    ring = ring.intersect(clip)
    leaf = leaf.union(ring)

    # Inner raised bead ring just inside the main ring.
    bead = (
        cq.Workplane("XZ", origin=(cx, front_y, cz))
        .circle(RING_INNER_R - 0.03)
        .circle(RING_INNER_R - 0.055)
        .extrude(-raise_h * 0.7)
    )
    bead = bead.intersect(clip)
    leaf = leaf.union(bead)

    return leaf


def _leaf_inset(sign: float) -> cq.Workplane:
    """Dark recessed inset disc that sits inside the central ring (the dark
    recessed motif), modeled in the leaf-local frame and trimmed to the leaf.
    Mirrored in X by `sign` so the two half-discs complete one disc."""
    body_w = LEAF_W - CENTER_REVEAL
    x_meet = sign * LEAF_W                 # nominal circle center
    x_body_edge = sign * body_w
    x_lo = min(0.0, x_body_edge)
    x_hi = max(0.0, x_body_edge)
    x_mid = (x_lo + x_hi) / 2.0
    cz = 0.0
    half_t = LEAF_T / 2.0
    clip = (
        cq.Workplane("XY")
        .box(body_w, LEAF_T, LEAF_H, centered=(True, True, True))
        .translate((x_mid, 0.0, 0.0))
    )
    disc = (
        cq.Workplane("XZ", origin=(x_meet, half_t - 0.004, cz))
        .circle(RING_INNER_R - 0.06)
        .extrude(-0.008)
    )
    disc = disc.intersect(clip)
    return disc


def _handle(sign: float) -> cq.Workplane:
    """Vertical pull handle: two standoffs and a vertical bar, modeled in the
    leaf-local frame near the meeting edge. Mirrored across X by `sign` and
    standing proud of the +Y front face for BOTH leaves (so both handles meet
    at the center on the same visible face)."""
    half_t = LEAF_T / 2.0
    x = sign * (LEAF_W - 0.10)   # near meeting edge, mirrored
    y_face = half_t - 0.002      # start just inside the front face
    standoff_len = 0.045
    y_bar = y_face + standoff_len  # bar centerline offset from the face
    bar_h = 0.34
    bar_r = 0.012
    post_r = 0.007

    # Vertical grab bar (along Z), offset out from the +Y face.
    result = (
        cq.Workplane("XY", origin=(x, y_bar, 0.0))
        .cylinder(bar_h, bar_r, direct=(0, 0, 1))
    )
    # Two standoff posts (along Y) joining the bar back to the leaf face.
    post_len = standoff_len + 0.010
    y_mid = y_face + post_len / 2.0 - 0.004
    for sz in (bar_h / 2 - 0.025, -(bar_h / 2 - 0.025)):
        post = (
            cq.Workplane("XY", origin=(x, y_mid, sz))
            .cylinder(post_len, post_r, direct=(0, 1, 0))
        )
        result = result.union(post, clean=True)
    return result


def _surround_members() -> list[tuple[str, cq.Workplane]]:
    """Dark steel surround built from individual CONVEX frame members so that
    derived (convex-hull) collisions never fill the door/sidelight/transom
    openings. Each member is its own named visual.

    World frame: door opening center X=0, sill top z=0.
    """
    d = FRAME_DEPTH
    half_w = SURROUND_W / 2.0
    head_z = LEAF_H + HEAD_RAIL_H + TRANSOM_H  # top of openings band
    top_z = head_z + JAMB_W

    members: list[tuple[str, cq.Workplane]] = []

    def member(name, sx, sz, cx, cz):
        members.append(
            (
                name,
                cq.Workplane("XY")
                .box(sx, d, sz, centered=(True, True, True))
                .translate((cx, 0.0, cz)),
            )
        )

    # Outer left and right jambs (full height).
    member("jamb_left", JAMB_W, top_z + JAMB_W, -half_w + JAMB_W / 2, (top_z - JAMB_W) / 2)
    member("jamb_right", JAMB_W, top_z + JAMB_W, half_w - JAMB_W / 2, (top_z - JAMB_W) / 2)

    # Sill (bottom) and head (top) rails spanning the full width.
    member("sill", SURROUND_W, JAMB_W, 0.0, -JAMB_W / 2)
    member("head", SURROUND_W, JAMB_W, 0.0, top_z - JAMB_W / 2)

    # Two mullions separating sidelights from the door opening.
    mull_x = DOOR_HALF + JAMB_W / 2
    for sgn, nm in ((-1.0, "mullion_left"), (1.0, "mullion_right")):
        member(nm, JAMB_W, LEAF_H + HEAD_RAIL_H + TRANSOM_H, sgn * mull_x,
               (LEAF_H + HEAD_RAIL_H + TRANSOM_H) / 2)

    # Head rail / mullion between the door leaves and the transom band.
    member("head_rail", OPENING_W, HEAD_RAIL_H, 0.0, LEAF_H + HEAD_RAIL_H / 2)

    return members


def _fixed_panels() -> list[cq.Workplane]:
    """Fixed dark recessed panels filling the sidelights and transom (these are
    part of the FIXED root, not operable). Panels are sized to slightly embed
    into the surrounding frame members so they read as seated/glazed-in and
    register as connected, not floating."""
    panels = []
    d = FRAME_DEPTH
    panel_t = 0.03
    panel_y = -d * 0.18  # set back into the surround
    embed = 0.012        # small seating embed into frame members

    half_w = SURROUND_W / 2.0
    mull_x = DOOR_HALF + JAMB_W / 2

    # Sidelight clear opening (in X) per side.
    sl_x0 = -half_w + JAMB_W
    sl_x1 = -(mull_x + JAMB_W / 2.0)
    sl_w = abs(sl_x1 - sl_x0) + 2 * embed
    sl_center_abs = abs((sl_x0 + sl_x1) / 2.0)
    sl_h = LEAF_H + 2 * embed
    for sign in (-1.0, 1.0):
        sl = (
            cq.Workplane("XY")
            .box(sl_w, panel_t, sl_h, centered=(True, True, True))
            .translate((sign * sl_center_abs, panel_y, LEAF_H / 2.0))
        )
        panels.append(sl)

    # Transom panel (above the head rail).
    transom_z0 = LEAF_H + HEAD_RAIL_H
    tr = (
        cq.Workplane("XY")
        .box(OPENING_W + 2 * embed, panel_t, TRANSOM_H + 2 * embed, centered=(True, True, True))
        .translate((0, panel_y, transom_z0 + TRANSOM_H / 2.0))
    )
    panels.append(tr)
    return panels


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ornate_double_door")
    for m in (WOOD, WOOD_DARK, STEEL, INSET_DARK, HANDLE):
        model.material(m.name, rgba=m.rgba)

    # --- Root: fixed dark-steel surround + fixed sidelight/transom panels ---
    surround = model.part("surround")
    for name, member in _surround_members():
        surround.visual(
            mesh_from_cadquery(member, f"surround_{name}"),
            material=STEEL,
            name=f"surround_{name}",
        )
    for i, panel in enumerate(_fixed_panels()):
        # i 0,1 = sidelights ; i 2 = transom
        nm = "sidelight_panel" if i < 2 else "transom_panel"
        surround.visual(
            mesh_from_cadquery(panel, f"{nm}_{i}"),
            material=INSET_DARK,
            name=f"{nm}_{i}",
        )

    # --- Two operable wood leaves (mirror-symmetric across X=0) ---
    # door_0: sign=+1 -> hinge edge at local x=0, body extends +X, meeting edge
    #   at local x=+LEAF_W. Placed at the LEFT jamb (world x = -DOOR_HALF), so
    #   the body reaches inward to the center (meeting edge -> world x=0).
    # door_1: sign=-1 -> hinge edge at local x=0, body extends -X, meeting edge
    #   at local x=-LEAF_W. Placed at the RIGHT jamb (world x = +DOOR_HALF), so
    #   the body reaches inward to the center (meeting edge -> world x=0).
    # Both front faces point +Y; mirroring is purely in X via `sign`.
    leaf_signs = {0: 1.0, 1: -1.0}
    for idx in range(2):
        sign = leaf_signs[idx]
        leaf = model.part(f"door_{idx}")
        leaf.visual(
            mesh_from_cadquery(_leaf_body(sign), f"door_body_{idx}"),
            material=WOOD,
            name=f"door_body_{idx}",
        )
        leaf.visual(
            mesh_from_cadquery(_leaf_inset(sign), f"door_inset_{idx}"),
            material=INSET_DARK,
            name=f"door_inset_{idx}",
        )
        leaf.visual(
            mesh_from_cadquery(_handle(sign), f"door_handle_{idx}"),
            material=HANDLE,
            name=f"door_handle_{idx}",
        )

    # Articulations: vertical hinge axes at the OUTER edges.
    # door_0: hinge at world x = -DOOR_HALF; body extends +X toward center, so
    #   the free meeting edge is on the +X side of the hinge. Rotating about +Z
    #   (axis=(0,0,1)) swings that free edge toward +Y (outward, toward the
    #   viewer) -> opens.
    model.articulation(
        "surround_to_door_0",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=model.get_part("door_0"),
        origin=Origin(xyz=(-DOOR_HALF, 0.0, LEAF_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.9),
    )
    # door_1: hinge at world x = +DOOR_HALF; body extends -X toward center, so
    #   the free meeting edge is on the -X side of the hinge. Rotating about -Z
    #   (axis=(0,0,-1)) swings that free edge toward +Y as well, the mirror of
    #   door_0.
    model.articulation(
        "surround_to_door_1",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=model.get_part("door_1"),
        origin=Origin(xyz=(DOOR_HALF, 0.0, LEAF_H / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.9),
    )
    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    surround = object_model.get_part("surround")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    j0 = object_model.get_articulation("surround_to_door_0")
    j1 = object_model.get_articulation("surround_to_door_1")

    # --- Hero geometry presence/scale ---
    # Two leaves exist at realistic leaf width and full height.
    for leaf in (door_0, door_1):
        aabb = ctx.part_world_aabb(leaf)
        assert aabb is not None
        (lo, hi) = aabb
        width = hi[0] - lo[0]
        height = hi[2] - lo[2]
        ctx.check(
            f"{leaf.name} leaf sized like a real door leaf",
            0.70 <= width <= 0.95 and 2.1 <= height <= 2.5,
            details=f"width={width:.3f} height={height:.3f}",
        )

    # Central raised-molding ring + dark recessed inset present on each leaf.
    for leaf, body, inset in (
        (door_0, "door_body_0", "door_inset_0"),
        (door_1, "door_body_1", "door_inset_1"),
    ):
        ring_aabb = ctx.part_element_world_aabb(leaf, elem=body)
        inset_aabb = ctx.part_element_world_aabb(leaf, elem=inset)
        assert ring_aabb is not None and inset_aabb is not None
        ring_h = ring_aabb[1][2] - ring_aabb[0][2]
        inset_w = inset_aabb[1][0] - inset_aabb[0][0]
        ctx.check(
            f"{leaf.name} carries a large circular motif",
            ring_h >= 1.0,
            details=f"ring body height={ring_h:.3f}",
        )
        ctx.check(
            f"{leaf.name} dark recessed inset disc present",
            inset_w >= 0.35,
            details=f"inset width={inset_w:.3f}",
        )

    # Pull handle present on each leaf.
    for leaf, handle in ((door_0, "door_handle_0"), (door_1, "door_handle_1")):
        h_aabb = ctx.part_element_world_aabb(leaf, elem=handle)
        assert h_aabb is not None
        h_height = h_aabb[1][2] - h_aabb[0][2]
        ctx.check(
            f"{leaf.name} has a vertical pull handle",
            h_height >= 0.20,
            details=f"handle height={h_height:.3f}",
        )

    # --- LEAF MIRROR SYMMETRY across the world X=0 plane (closed pose) ---
    # The user complaint was that the leaves were not mirror-symmetric: the
    # decorated faces (ring/inset/handle) must match and lie on the same +Y
    # front face, and each leaf's geometry must be the X-mirror of the other.
    with ctx.pose({j0: 0.0, j1: 0.0}):
        def _aabb(part, elem=None):
            a = (
                ctx.part_element_world_aabb(part, elem=elem)
                if elem is not None
                else ctx.part_world_aabb(part)
            )
            assert a is not None
            return a

        def _center(a):
            lo, hi = a
            return [(lo[i] + hi[i]) / 2.0 for i in range(3)]

        def _dims(a):
            lo, hi = a
            return [hi[i] - lo[i] for i in range(3)]

        # Whole-leaf mirror: centers mirror in X, match in Y and Z; dims equal.
        a0 = _aabb(door_0)
        a1 = _aabb(door_1)
        c0, c1 = _center(a0), _center(a1)
        d0d, d1d = _dims(a0), _dims(a1)
        ctx.check(
            "leaves mirror in X (centers opposite, equal magnitude)",
            abs(c0[0] + c1[0]) < 0.01 and c0[0] < 0 < c1[0],
            details=f"cx0={c0[0]:.4f} cx1={c1[0]:.4f}",
        )
        ctx.check(
            "leaves share the same depth (Y) center (both faces forward)",
            abs(c0[1] - c1[1]) < 0.005,
            details=f"cy0={c0[1]:.4f} cy1={c1[1]:.4f}",
        )
        ctx.check(
            "leaves share the same height (Z) center",
            abs(c0[2] - c1[2]) < 0.005,
            details=f"cz0={c0[2]:.4f} cz1={c1[2]:.4f}",
        )
        ctx.check(
            "leaves have identical dimensions",
            all(abs(d0d[i] - d1d[i]) < 0.005 for i in range(3)),
            details=f"d0={[round(v,4) for v in d0d]} d1={[round(v,4) for v in d1d]}",
        )

        # Decorated-element mirror: each of ring/inset/handle must mirror in X
        # AND sit on the same front face (matching Y center) so the grain and
        # the circle line up.
        for elem in ("door_body", "door_inset", "door_handle"):
            e0 = _aabb(door_0, elem=f"{elem}_0")
            e1 = _aabb(door_1, elem=f"{elem}_1")
            ce0, ce1 = _center(e0), _center(e1)
            de0, de1 = _dims(e0), _dims(e1)
            ctx.check(
                f"{elem} mirrors across X=0",
                abs(ce0[0] + ce1[0]) < 0.01,
                details=f"cx0={ce0[0]:.4f} cx1={ce1[0]:.4f}",
            )
            ctx.check(
                f"{elem} sits on the same front face (matching Y)",
                abs(ce0[1] - ce1[1]) < 0.01,
                details=f"cy0={ce0[1]:.4f} cy1={ce1[1]:.4f}",
            )
            ctx.check(
                f"{elem} has equal size on both leaves",
                all(abs(de0[i] - de1[i]) < 0.01 for i in range(3)),
                details=f"d0={[round(v,4) for v in de0]} d1={[round(v,4) for v in de1]}",
            )

        # The two half-rings complete ONE circle centered on the meeting edge
        # at world x=0: the body inner edges must meet at the center.
        ctx.check(
            "half-rings meet at the central X=0 meeting edge",
            a0[1][0] >= -0.02 and a1[0][0] <= 0.02,
            details=f"door_0.maxx={a0[1][0]:.4f} door_1.minx={a1[0][0]:.4f}",
        )

        # Leaves meet at center without interpenetrating.
        ctx.expect_gap(door_1, door_0, axis="x", min_gap=-0.01, max_gap=0.03)
        # Leaves seated within the door opening height.
        ctx.expect_within(door_0, surround, axes="z", margin=0.05)
        ctx.expect_within(door_1, surround, axes="z", margin=0.05)

    # --- Fixed elements stay put: sidelights + transom belong to the root ---
    surround_aabb = ctx.part_world_aabb(surround)
    assert surround_aabb is not None
    s_w = surround_aabb[1][0] - surround_aabb[0][0]
    s_h = surround_aabb[1][2] - surround_aabb[0][2]
    ctx.check(
        "surround spans wider than the door opening (sidelights present)",
        s_w >= OPENING_W + 2.0 * SIDELIGHT_W,
        details=f"surround width={s_w:.3f} opening={OPENING_W:.3f}",
    )
    ctx.check(
        "surround rises above the leaves (transom present)",
        s_h >= LEAF_H + TRANSOM_H,
        details=f"surround height={s_h:.3f}",
    )
    side0 = ctx.part_element_world_aabb(surround, elem="sidelight_panel_0")
    side1 = ctx.part_element_world_aabb(surround, elem="sidelight_panel_1")
    transom = ctx.part_element_world_aabb(surround, elem="transom_panel_2")
    assert side0 is not None and side1 is not None and transom is not None
    ctx.check(
        "two fixed sidelight panels flank the doors",
        side0[1][0] < -DOOR_HALF + 0.01 and side1[0][0] > DOOR_HALF - 0.01,
        details=f"side0.maxx={side0[1][0]:.3f} side1.minx={side1[0][0]:.3f}",
    )
    ctx.check(
        "fixed transom panel sits above the leaves",
        transom[0][2] > LEAF_H - 0.01,
        details=f"transom.minz={transom[0][2]:.3f}",
    )

    # --- Open pose: both leaves swing clear toward +Y, hinges stay connected ---
    def _body_y(leaf, elem):
        aabb = ctx.part_element_world_aabb(leaf, elem=elem)
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    closed_y0 = _body_y(door_0, "door_body_0")
    closed_y1 = _body_y(door_1, "door_body_1")
    with ctx.pose({j0: 1.6, j1: 1.6}):
        open_y0 = _body_y(door_0, "door_body_0")
        open_y1 = _body_y(door_1, "door_body_1")
        # Hinge edges remain attached to the surround at the outer jambs.
        ctx.expect_contact(door_0, surround, name="door_0 hinge stays attached")
        ctx.expect_contact(door_1, surround, name="door_1 hinge stays attached")
        # Open leaves remain mirror-symmetric (both swing to +Y equally).
        ctx.check(
            "open leaves stay mirror-symmetric in depth",
            open_y0 is not None and open_y1 is not None and abs(open_y0 - open_y1) < 0.02,
            details=f"open_y0={open_y0} open_y1={open_y1}",
        )
    assert None not in (closed_y0, closed_y1, open_y0, open_y1)
    ctx.check(
        "door_0 swings outward (toward viewer) when opened",
        open_y0 > closed_y0 + 0.20,
        details=f"closed_y={closed_y0:.3f} open_y={open_y0:.3f}",
    )
    ctx.check(
        "door_1 swings outward (toward viewer) when opened",
        open_y1 > closed_y1 + 0.20,
        details=f"closed_y={closed_y1:.3f} open_y={open_y1:.3f}",
    )

    return ctx.report()
