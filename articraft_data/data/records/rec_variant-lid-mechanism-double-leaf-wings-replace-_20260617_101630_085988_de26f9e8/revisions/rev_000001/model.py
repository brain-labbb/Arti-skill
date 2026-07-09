from __future__ import annotations

"""Rustic toe-pincher wooden coffin with double-leaf wing lid.

A hollow six-sided (toe-pincher) plank box about 1.9 m long, 0.6 m wide at the
shoulders, tapering to ~0.35 m at the head and ~0.3 m at the foot, ~0.4 m tall
overall. Three darker strap boards wrap across each lid leaf and down the body
sides (near the head, at the shoulder break, near the foot), and a dark Latin
cross sits on the head-end region of one lid leaf. The lid is split into two
long leaves that meet at the centerline; each leaf hinges on its own long side
rim edge and swings open like wings (0..110 degrees each) to reveal the hollow
interior.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Dimensions (meters). Coffin lies along +X, head at -X, foot at +X, z up.
# ----------------------------------------------------------------------------
LENGTH = 1.90
HEAD_HALF_W = 0.175  # ~0.35 m across the head end
SHOULDER_HALF_W = 0.30  # ~0.60 m across the shoulders
FOOT_HALF_W = 0.15  # ~0.30 m across the foot end
HEAD_X = -LENGTH / 2.0
FOOT_X = LENGTH / 2.0
SHOULDER_X = HEAD_X + LENGTH / 3.0  # shoulder break ~1/3 from the head end

WALL_T = 0.03
FLOOR_T = 0.03
BODY_H = 0.33  # box height to the open rim
LID_T = 0.04  # lid plank panel thickness
LID_OVERHANG = 0.015  # lid outline overhangs the box rim
STRAP_T = 0.012  # strap boards stand proud by this much
STRAP_W = 0.08  # strap board width along the coffin length
STRAP_EMBED = 0.003  # straps sink slightly into their carrier plank face

HINGE_ABS = SHOULDER_HALF_W + LID_OVERHANG  # distance from centerline to hinge line
LID_OPEN_MAX = math.radians(110.0)

# Strap board x stations: near head, at the shoulder break, near the foot.
STRAP_XS = {"head_strap": -0.75, "shoulder_strap": SHOULDER_X, "foot_strap": 0.65}

NUM_LEAVES = 2

# Toe-pincher footprint, counter-clockwise.
FOOTPRINT = (
    (HEAD_X, -HEAD_HALF_W),
    (SHOULDER_X, -SHOULDER_HALF_W),
    (FOOT_X, -FOOT_HALF_W),
    (FOOT_X, FOOT_HALF_W),
    (SHOULDER_X, SHOULDER_HALF_W),
    (HEAD_X, HEAD_HALF_W),
)


def _offset_polygon(pts, dist):
    """Offset a convex CCW polygon: positive dist moves edges outward."""
    n = len(pts)
    lines = []
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        dx, dy = dx / length, dy / length
        # Outward normal of a CCW polygon edge is (dy, -dx).
        lines.append((x1 + dy * dist, y1 - dx * dist, dx, dy))
    out = []
    for i in range(n):
        ax, ay, adx, ady = lines[i - 1]
        bx, by, bdx, bdy = lines[i]
        denom = adx * bdy - ady * bdx
        s = ((bx - ax) * bdy - (by - ay) * bdx) / denom
        out.append((ax + adx * s, ay + ady * s))
    return out


def _clip_polygon_at_y0(pts, keep_positive):
    """Clip a convex CCW polygon to y >= 0 or y <= 0 (Sutherland-Hodgman)."""
    out = []
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        inside1 = (y1 >= -1e-12) if keep_positive else (y1 <= 1e-12)
        inside2 = (y2 >= -1e-12) if keep_positive else (y2 <= 1e-12)
        if inside1:
            out.append((x1, y1))
        if inside1 != inside2:
            dy = y2 - y1
            t = -y1 / dy if abs(dy) > 1e-15 else 0.0
            out.append((x1 + t * (x2 - x1), 0.0))
    return out


def _clip_polygon_y_max(pts, y_max):
    """Clip a convex CCW polygon to y <= y_max."""
    shifted = [(x, y - y_max) for x, y in pts]
    clipped = _clip_polygon_at_y0(shifted, keep_positive=False)
    return [(x, y + y_max) for x, y in clipped]


def _clip_polygon_y_min(pts, y_min):
    """Clip a convex CCW polygon to y >= y_min."""
    shifted = [(x, y - y_min) for x, y in pts]
    clipped = _clip_polygon_at_y0(shifted, keep_positive=True)
    return [(x, y + y_min) for x, y in clipped]


def _extrude_poly(pts, z0, z1):
    """Extrude a closed 2D polygon between two z heights."""
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .polyline(list(pts))
        .close()
        .extrude(z1 - z0)
    )


def _x_slab(x_center, width, z_center=0.25, z_height=0.9):
    """Wide y-spanning slab used to crop wrap bands to one strap station."""
    return cq.Workplane("XY", origin=(x_center, 0.0, z_center)).box(width, 2.0, z_height)


def _body_shell():
    """Hollow toe-pincher box: open top, 0.03 m walls and floor, plank seams."""
    outer = _extrude_poly(FOOTPRINT, 0.0, BODY_H)
    cavity = _extrude_poly(_offset_polygon(FOOTPRINT, -WALL_T), FLOOR_T, BODY_H + 0.01)
    shell = outer.cut(cavity)
    # Two shallow horizontal seam grooves around the outside: three plank rows.
    for z_seam in (0.11, 0.22):
        ring = _extrude_poly(FOOTPRINT, z_seam - 0.0025, z_seam + 0.0025).cut(
            _extrude_poly(
                _offset_polygon(FOOTPRINT, -0.005), z_seam - 0.004, z_seam + 0.004
            )
        )
        shell = shell.cut(ring)
    return shell


def _body_strap_band(x_center):
    """Darker strap board wrapping the angled body side walls at one station."""
    ring = _extrude_poly(_offset_polygon(FOOTPRINT, STRAP_T), 0.02, BODY_H).cut(
        _extrude_poly(_offset_polygon(FOOTPRINT, -0.002), 0.015, BODY_H + 0.01)
    )
    return ring.intersect(_x_slab(x_center, STRAP_W))


# ---------------------------------------------------------------------------
# Lid leaf geometry helpers (shared across both leaves via a for loop).
# ---------------------------------------------------------------------------

def _compute_leaf_local_polygons():
    """Return (neg_local_poly, pos_local_poly) for the two lid leaves.

    Each leaf polygon is in its own local frame with the hinge edge near
    y=0 and the panel extending away from the hinge (leaf 0 along +Y,
    leaf 1 along -Y).
    """
    overhang = _offset_polygon(FOOTPRINT, LID_OVERHANG)
    neg_half = _clip_polygon_at_y0(overhang, keep_positive=False)
    pos_half = _clip_polygon_at_y0(overhang, keep_positive=True)
    # Leaf 0 (-Y side): hinge at y=-HINGE_ABS, shift so hinge edge is at local y=0
    neg_local = [(x, y + HINGE_ABS) for x, y in neg_half]
    # Leaf 1 (+Y side): hinge at y=+HINGE_ABS, shift so hinge edge is at local y=0
    pos_local = [(x, y - HINGE_ABS) for x, y in pos_half]
    return neg_local, pos_local


def _lid_leaf_panel(local_poly):
    """Flat lid leaf plank panel with a lengthwise plank seam groove."""
    panel = _extrude_poly(local_poly, 0.0, LID_T)
    # One shallow seam groove across the leaf width (mid-leaf)
    ys = [y for _, y in local_poly]
    y_min, y_max = min(ys), max(ys)
    y_mid = (y_min + y_max) / 2.0
    groove = cq.Workplane("XY", origin=(0.0, y_mid, LID_T)).box(2.2, 0.004, 0.008)
    panel = panel.cut(groove)
    return panel


def _lid_leaf_strap(local_poly, x_center, y_clip_max=None, y_clip_min=None):
    """Darker strap board lying across one lid leaf at one station.

    y_clip_max/y_clip_min clip the offset polygon at the centerline so
    opposing straps from the two leaves meet exactly without interpenetration.
    """
    offset_poly = _offset_polygon(local_poly, STRAP_T)
    if y_clip_max is not None:
        offset_poly = _clip_polygon_y_max(offset_poly, y_clip_max)
    if y_clip_min is not None:
        offset_poly = _clip_polygon_y_min(offset_poly, y_clip_min)
    inner_poly = _offset_polygon(local_poly, -0.002)
    cap = _extrude_poly(offset_poly, 0.0, LID_T + STRAP_T).cut(
        _extrude_poly(inner_poly, -0.01, LID_T - STRAP_EMBED)
    )
    return cap.intersect(_x_slab(x_center, STRAP_W, z_center=0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toe_pincher_coffin_double_leaf")

    plank_wood = model.material("plank_wood", rgba=(0.44, 0.31, 0.20, 1.0))
    strap_wood = model.material("strap_wood", rgba=(0.29, 0.19, 0.12, 1.0))
    cross_wood = model.material("cross_wood", rgba=(0.22, 0.14, 0.09, 1.0))

    # ---- hollow box body (identical to parent) ----------------------------
    body = model.part("coffin_body")
    shell = _body_shell()
    body.visual(mesh_from_cadquery(shell, "body_shell"), material=plank_wood, name="body_shell")
    for strap_name, x_center in STRAP_XS.items():
        body.visual(
            mesh_from_cadquery(_body_strap_band(x_center), f"body_{strap_name}"),
            material=strap_wood,
            name=f"body_{strap_name}",
        )

    # ---- double-leaf wing lid (two leaves via a for loop) -----------------
    neg_local, pos_local = _compute_leaf_local_polygons()

    # Leaf 0: -Y side, hinge at world y = -HINGE_ABS, axis +X (positive q lifts +Y edge up)
    # Leaf 1: +Y side, hinge at world y = +HINGE_ABS, axis -X (positive q lifts -Y edge up)
    leaf_configs = [
        {"local_poly": neg_local, "hinge_y": -HINGE_ABS, "axis": (1.0, 0.0, 0.0),
         "strap_y_clip": {"y_clip_max": HINGE_ABS}},
        {"local_poly": pos_local, "hinge_y": HINGE_ABS, "axis": (-1.0, 0.0, 0.0),
         "strap_y_clip": {"y_clip_min": -HINGE_ABS}},
    ]

    lid_leaf_parts = []
    lid_leaf_hinges = []

    for i in range(NUM_LEAVES):
        cfg = leaf_configs[i]
        leaf_name = f"lid_leaf_{i}"
        leaf = model.part(leaf_name)
        lid_leaf_parts.append(leaf)

        # Lid leaf panel
        leaf.visual(
            mesh_from_cadquery(_lid_leaf_panel(cfg["local_poly"]), f"lid_leaf_{i}_panel"),
            material=plank_wood,
            name=f"lid_leaf_{i}_panel",
        )

        # Strap boards on this leaf (clipped at centerline to avoid cross-leaf overlap)
        for strap_name, x_center in STRAP_XS.items():
            leaf.visual(
                mesh_from_cadquery(
                    _lid_leaf_strap(cfg["local_poly"], x_center, **cfg["strap_y_clip"]),
                    f"lid_leaf_{i}_{strap_name}",
                ),
                material=strap_wood,
                name=f"lid_leaf_{i}_{strap_name}",
            )

        # Latin cross on leaf 0 only (head-end region, slightly inset from centerline edge)
        if i == 0:
            # In leaf_0 local frame, centerline edge is at y ≈ HINGE_ABS.
            # Offset cross slightly inward so the arm fits within the leaf.
            cross_y = HINGE_ABS - 0.025
            cross_z = LID_T - STRAP_EMBED + 0.0075
            leaf.visual(
                Box((0.32, 0.05, 0.015)),
                origin=Origin(xyz=(-0.54, cross_y, cross_z)),
                material=cross_wood,
                name="cross_upright",
            )
            leaf.visual(
                Box((0.05, 0.18, 0.015)),
                origin=Origin(xyz=(-0.61, cross_y - 0.06, cross_z)),
                material=cross_wood,
                name="cross_arm",
            )

        # Revolute hinge at the rim edge
        hinge = model.articulation(
            f"lid_leaf_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leaf,
            origin=Origin(xyz=(0.0, cfg["hinge_y"], BODY_H)),
            axis=cfg["axis"],
            motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=0.0, upper=LID_OPEN_MAX),
        )
        lid_leaf_hinges.append(hinge)

    # Record hollowness evidence for prompt-specific tests.
    outer_volume = float(_extrude_poly(FOOTPRINT, 0.0, BODY_H).val().Volume())
    shell_volume = float(shell.val().Volume())
    model.meta["outer_solid_volume"] = outer_volume
    model.meta["shell_volume"] = shell_volume

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("coffin_body")
    leaf_0 = object_model.get_part("lid_leaf_0")
    leaf_1 = object_model.get_part("lid_leaf_1")
    hinge_0 = object_model.get_articulation("lid_leaf_0_hinge")
    hinge_1 = object_model.get_articulation("lid_leaf_1_hinge")

    # --- overall toe-pincher dimensions and grounding -----------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body footprint is ~1.9 m long and ~0.6 m wide at the shoulders",
        body_aabb is not None
        and abs((body_aabb[1][0] - body_aabb[0][0]) - LENGTH) < 0.05
        and abs((body_aabb[1][1] - body_aabb[0][1]) - 2.0 * SHOULDER_HALF_W) < 0.05,
        details=f"body aabb={body_aabb}",
    )
    ctx.check(
        "box sits on the ground and rim is at ~0.33 m",
        body_aabb is not None
        and abs(body_aabb[0][2]) < 0.005
        and abs(body_aabb[1][2] - BODY_H) < 0.01,
        details=f"body aabb={body_aabb}",
    )

    # --- closed lid pair covers the opening ---------------------------------
    leaf_0_aabb = ctx.part_world_aabb(leaf_0)
    leaf_1_aabb = ctx.part_world_aabb(leaf_1)
    lid_top = BODY_H + LID_T + STRAP_T  # straps stand proud above the panel
    ctx.check(
        "closed leaf 0 top is at ~0.38 m (BODY_H + LID_T + STRAP_T)",
        leaf_0_aabb is not None and abs(leaf_0_aabb[1][2] - lid_top) < 0.01,
        details=f"leaf_0 aabb={leaf_0_aabb}",
    )
    ctx.check(
        "closed leaf 1 top is at ~0.38 m (BODY_H + LID_T + STRAP_T)",
        leaf_1_aabb is not None and abs(leaf_1_aabb[1][2] - lid_top) < 0.01,
        details=f"leaf_1 aabb={leaf_1_aabb}",
    )

    # Two leaves together should cover the full coffin width
    ctx.check(
        "two closed leaves span the full coffin width (~0.63 m)",
        leaf_0_aabb is not None
        and leaf_1_aabb is not None
        and abs((max(leaf_0_aabb[1][1], leaf_1_aabb[1][1]) - min(leaf_0_aabb[0][1], leaf_1_aabb[0][1])) - 2.0 * HINGE_ABS) < 0.03,
        details=f"leaf_0 y=[{leaf_0_aabb[0][1]:.3f},{leaf_0_aabb[1][1]:.3f}], leaf_1 y=[{leaf_1_aabb[0][1]:.3f},{leaf_1_aabb[1][1]:.3f}]",
    )

    # --- toe-pincher taper on the body shell --------------------------------
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "shoulder width exceeds head and foot widths (toe-pincher taper)",
        shell_aabb is not None
        and 2.0 * SHOULDER_HALF_W > 2.0 * HEAD_HALF_W > 2.0 * FOOT_HALF_W
        and abs((shell_aabb[1][1] - shell_aabb[0][1]) - 2.0 * SHOULDER_HALF_W) < 0.01,
        details=f"shell aabb={shell_aabb}",
    )

    # --- hollow interior with ~0.03 m walls --------------------------------
    outer_volume = float(object_model.meta["outer_solid_volume"])
    shell_volume = float(object_model.meta["shell_volume"])
    ctx.check(
        "box is a hollow open-top shell, not a solid block",
        0.0 < shell_volume < 0.45 * outer_volume,
        details=f"shell={shell_volume:.4f} m^3, outer={outer_volume:.4f} m^3",
    )

    # --- closed leaves seat on the rim --------------------------------------
    ctx.expect_gap(leaf_0, body, axis="z", max_gap=0.002, max_penetration=0.0005,
                   positive_elem="lid_leaf_0_panel",
                   name="leaf 0 panel seats on body rim")
    ctx.expect_gap(leaf_1, body, axis="z", max_gap=0.002, max_penetration=0.0005,
                   positive_elem="lid_leaf_1_panel",
                   name="leaf 1 panel seats on body rim")

    # --- opposing straps meet at the centerline without overlapping ----------
    for strap_name, _x_center in STRAP_XS.items():
        ctx.expect_contact(
            leaf_0, leaf_1,
            elem_a=f"lid_leaf_0_{strap_name}",
            elem_b=f"lid_leaf_1_{strap_name}",
            contact_tol=0.002,
            name=f"opposing {strap_name} boards meet at the centerline",
        )

    # --- two revolute hinges: mirrored pair ---------------------------------
    ctx.check(
        "leaf 0 hinge is revolute with range 0 to ~110 degrees",
        hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_0.motion_limits is not None
        and abs(hinge_0.motion_limits.lower) < 1e-9
        and abs(hinge_0.motion_limits.upper - LID_OPEN_MAX) < 0.02,
        details=f"type={hinge_0.articulation_type}, limits=({hinge_0.motion_limits.lower}, {hinge_0.motion_limits.upper})",
    )
    ctx.check(
        "leaf 1 hinge is revolute with range 0 to ~110 degrees",
        hinge_1.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.motion_limits is not None
        and abs(hinge_1.motion_limits.lower) < 1e-9
        and abs(hinge_1.motion_limits.upper - LID_OPEN_MAX) < 0.02,
        details=f"type={hinge_1.articulation_type}, limits=({hinge_1.motion_limits.lower}, {hinge_1.motion_limits.upper})",
    )

    # Axes: leaf 0 along +X, leaf 1 along -X (mirrored pair)
    ctx.check(
        "leaf 0 hinge axis runs along +X (coffin length)",
        abs(hinge_0.axis[0] - 1.0) < 1e-6 and abs(hinge_0.axis[1]) < 1e-6 and abs(hinge_0.axis[2]) < 1e-6,
        details=f"axis={hinge_0.axis}",
    )
    ctx.check(
        "leaf 1 hinge axis runs along -X (mirrored pair)",
        abs(hinge_1.axis[0] + 1.0) < 1e-6 and abs(hinge_1.axis[1]) < 1e-6 and abs(hinge_1.axis[2]) < 1e-6,
        details=f"axis={hinge_1.axis}",
    )

    # Hinge origins on opposite side rim edges
    ctx.check(
        "hinges are on opposite side rim edges (mirrored y positions)",
        abs(hinge_0.origin.xyz[1] + HINGE_ABS) < 0.01
        and abs(hinge_1.origin.xyz[1] - HINGE_ABS) < 0.01
        and abs(hinge_0.origin.xyz[2] - BODY_H) < 0.005
        and abs(hinge_1.origin.xyz[2] - BODY_H) < 0.005,
        details=f"h0 origin={hinge_0.origin.xyz}, h1 origin={hinge_1.origin.xyz}",
    )

    # --- open both leaves: wings spread and reveal interior ------------------
    cross_closed = ctx.part_element_world_aabb(leaf_0, elem="cross_upright")
    with ctx.pose({hinge_0: LID_OPEN_MAX, hinge_1: LID_OPEN_MAX}):
        open_0 = ctx.part_world_aabb(leaf_0)
        open_1 = ctx.part_world_aabb(leaf_1)
        ctx.check(
            "leaf 0 swings up past vertical when fully open",
            open_0 is not None and open_0[1][2] > 0.55,
            details=f"open leaf_0 aabb={open_0}",
        )
        ctx.check(
            "leaf 1 swings up past vertical when fully open",
            open_1 is not None and open_1[1][2] > 0.55,
            details=f"open leaf_1 aabb={open_1}",
        )
        ctx.check(
            "open leaves spread outward past the body sides (wing configuration)",
            open_0 is not None
            and open_1 is not None
            and open_0[0][1] < -(SHOULDER_HALF_W + 0.10)
            and open_1[1][1] > SHOULDER_HALF_W + 0.10,
            details=f"open leaf_0 y_min={open_0[0][1]:.3f}, open leaf_1 y_max={open_1[1][1]:.3f}",
        )
        cross_open = ctx.part_element_world_aabb(leaf_0, elem="cross_upright")
        ctx.check(
            "cross rides up with leaf 0 through the hinge rotation",
            cross_closed is not None
            and cross_open is not None
            and cross_open[0][2] > cross_closed[1][2] + 0.10,
            details=f"closed={cross_closed}, open={cross_open}",
        )

    # --- cross sits on leaf 0 near the head-end centerline ------------------
    ctx.check(
        "Latin cross sits proud on leaf 0 head-end panel",
        cross_closed is not None
        and cross_closed[1][2] > BODY_H + LID_T + 0.005
        and cross_closed[1][0] < SHOULDER_X
        and cross_closed[0][1] < 0.05,
        details=f"cross aabb={cross_closed}",
    )
    cross_arm = ctx.part_element_world_aabb(leaf_0, elem="cross_arm")
    ctx.check(
        "cross arm spans wider than 0.12 m along y on leaf 0",
        cross_arm is not None
        and (cross_arm[1][1] - cross_arm[0][1]) > 0.12
        and cross_arm[0][0] < cross_closed[0][0] + 0.15,
        details=f"arm aabb={cross_arm}",
    )

    # --- three strap boards on each leaf ------------------------------------
    for i in range(NUM_LEAVES):
        for strap_name, x_center in STRAP_XS.items():
            strap_aabb = ctx.part_element_world_aabb(
                object_model.get_part(f"lid_leaf_{i}"),
                elem=f"lid_leaf_{i}_{strap_name}",
            )
            ctx.check(
                f"leaf {i} {strap_name} board at x~{x_center:+.2f}",
                strap_aabb is not None
                and abs((strap_aabb[0][0] + strap_aabb[1][0]) / 2.0 - x_center) < 0.03
                and (strap_aabb[1][1] - strap_aabb[0][1]) > 0.10
                and strap_aabb[1][2] > BODY_H + LID_T + 0.003,
                details=f"strap aabb={strap_aabb}",
            )

    # --- body strap boards wrap the side walls ------------------------------
    for strap_name, x_center in STRAP_XS.items():
        body_band = ctx.part_element_world_aabb(body, elem=f"body_{strap_name}")
        ctx.check(
            f"{strap_name} body band wraps down both side walls",
            body_band is not None
            and abs((body_band[0][0] + body_band[1][0]) / 2.0 - x_center) < 0.02
            and (body_band[1][2] - body_band[0][2]) > 0.25
            and (body_band[1][1] - body_band[0][1]) > 0.40,
            details=f"body band aabb={body_band}",
        )

    return ctx.report()


object_model = build_object_model()
