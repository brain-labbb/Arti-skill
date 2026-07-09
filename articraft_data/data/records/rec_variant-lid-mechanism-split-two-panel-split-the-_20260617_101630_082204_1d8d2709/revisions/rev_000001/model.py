from __future__ import annotations

"""Rustic toe-pincher wooden coffin with a split two-panel lid.

A hollow six-sided (toe-pincher) plank box about 1.9 m long, 0.6 m wide at the
shoulders, tapering to ~0.35 m at the head and ~0.3 m at the foot, ~0.4 m tall
overall. Three darker strap boards wrap across the lid panels and down the body
sides (near the head, at the shoulder break, near the foot), and a dark Latin
cross sits on the head-end lid panel. The lid is split into two independent
flat hexagonal plank panels — a head-half (lid_0) and a foot-half (lid_1) —
each hinged along the same long-side rim edge at the top of the -Y side wall,
swinging open 0..110 degrees independently.
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

HINGE_Y = -(SHOULDER_HALF_W + LID_OVERHANG)  # hinge line along the -Y rim edge
LID_OPEN_MAX = math.radians(110.0)

# Lid split: two panels divided at the coffin center with a small visible gap.
LID_SPLIT_X = 0.0
LID_GAP = 0.005  # visible gap between the two lid panels

# Strap board x stations: near head, at the shoulder break, near the foot.
STRAP_XS = {"head_strap": -0.75, "shoulder_strap": SHOULDER_X, "foot_strap": 0.65}

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


def _x_clip_box(x_min, x_max, y_center=0.30, y_extent=2.0, z_center=0.02, z_extent=0.20):
    """Box for clipping lid geometry to an x range."""
    return (
        cq.Workplane("XY", origin=((x_min + x_max) / 2.0, y_center, z_center))
        .box(x_max - x_min, y_extent, z_extent)
    )


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


# Lid local frame: origin on the hinge line (world (0, HINGE_Y, BODY_H)),
# panel extends along local +Y, plank top toward local +Z.
LID_POLY = tuple(
    (x, y - HINGE_Y) for (x, y) in _offset_polygon(FOOTPRINT, LID_OVERHANG)
)

# Panel x ranges in the lid local frame.
LID_PANEL_X_RANGES = [
    (HEAD_X - LID_OVERHANG - 0.01, LID_SPLIT_X - LID_GAP / 2.0),  # lid_0: head
    (LID_SPLIT_X + LID_GAP / 2.0, FOOT_X + LID_OVERHANG + 0.01),  # lid_1: foot
]


def _lid_panel_half(x_min, x_max):
    """Flat hexagonal lid plank panel clipped to [x_min, x_max]."""
    panel = _extrude_poly(LID_POLY, 0.0, LID_T)
    # Three lengthwise planks: two shallow seam grooves across the top face.
    lid_w = 2.0 * (SHOULDER_HALF_W + LID_OVERHANG)
    for y_seam in (lid_w / 3.0, 2.0 * lid_w / 3.0):
        groove = cq.Workplane("XY", origin=(0.0, y_seam, LID_T)).box(2.2, 0.004, 0.008)
        panel = panel.cut(groove)
    # Clip to this panel's x range.
    return panel.intersect(_x_clip_box(x_min, x_max))


def _lid_strap(x_center):
    """Darker strap board lying across the lid and wrapping over its edges."""
    cap = _extrude_poly(_offset_polygon(LID_POLY, STRAP_T), 0.0, LID_T + STRAP_T).cut(
        _extrude_poly(_offset_polygon(LID_POLY, -0.002), -0.01, LID_T - STRAP_EMBED)
    )
    return cap.intersect(_x_slab(x_center, STRAP_W, z_center=0.0))


# Pre-assign each strap to the panel whose x range contains it.
LID_PANEL_STRAPS: list[list[tuple[str, float]]] = [[], []]
for _sname, _sx in STRAP_XS.items():
    for _pi, (_xmin, _xmax) in enumerate(LID_PANEL_X_RANGES):
        if _xmin - 0.05 <= _sx <= _xmax + 0.05:
            LID_PANEL_STRAPS[_pi].append((_sname, _sx))
            break


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toe_pincher_coffin_split_lid")

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

    # ---- split two-panel lid ----------------------------------------------
    lid_parts = []
    hinge_names = []

    for i in range(2):
        x_min, x_max = LID_PANEL_X_RANGES[i]
        lid = model.part(f"lid_{i}")

        # Panel plank with seam grooves.
        lid.visual(
            mesh_from_cadquery(_lid_panel_half(x_min, x_max), f"lid_panel_{i}"),
            material=plank_wood,
            name=f"lid_panel_{i}",
        )

        # Strap boards on this panel.
        for j, (strap_name, x_center) in enumerate(LID_PANEL_STRAPS[i]):
            lid.visual(
                mesh_from_cadquery(_lid_strap(x_center), f"lid_strap_{i}_{j}"),
                material=strap_wood,
                name=f"lid_strap_{i}_{j}",
            )

        # Latin cross on the head-end panel (lid_0) only.
        if i == 0:
            lid_center_y = SHOULDER_HALF_W + LID_OVERHANG  # lid local y of centerline
            cross_z = LID_T - STRAP_EMBED + 0.0075
            lid.visual(
                Box((0.32, 0.05, 0.015)),
                origin=Origin(xyz=(-0.54, lid_center_y, cross_z)),
                material=cross_wood,
                name="cross_upright",
            )
            lid.visual(
                Box((0.05, 0.20, 0.015)),
                origin=Origin(xyz=(-0.61, lid_center_y, cross_z)),
                material=cross_wood,
                name="cross_arm",
            )

        # Hinge along the coffin's length at the top rim of the -Y side wall.
        # Closed panel extends along local +Y, so positive q about +X lifts the
        # free edge upward (right-hand rule) and swings it open toward -Y.
        hinge_name = f"lid_hinge_{i}"
        model.articulation(
            hinge_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=lid,
            origin=Origin(xyz=(0.0, HINGE_Y, BODY_H)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0, velocity=1.0, lower=0.0, upper=LID_OPEN_MAX
            ),
        )

        lid_parts.append(lid)
        hinge_names.append(hinge_name)

    # Record hollowness evidence for prompt-specific tests.
    outer_volume = float(_extrude_poly(FOOTPRINT, 0.0, BODY_H).val().Volume())
    shell_volume = float(shell.val().Volume())
    model.meta["outer_solid_volume"] = outer_volume
    model.meta["shell_volume"] = shell_volume

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("coffin_body")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    hinge_0 = object_model.get_articulation("lid_hinge_0")
    hinge_1 = object_model.get_articulation("lid_hinge_1")

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

    # --- hollow interior with ~0.03 m walls --------------------------------
    outer_volume = float(object_model.meta["outer_solid_volume"])
    shell_volume = float(object_model.meta["shell_volume"])
    ctx.check(
        "box is a hollow open-top shell, not a solid block",
        0.0 < shell_volume < 0.45 * outer_volume,
        details=f"shell={shell_volume:.4f} m^3, outer={outer_volume:.4f} m^3",
    )

    # --- two lid panels exist and cover the coffin opening ------------------
    for i, lid in enumerate((lid_0, lid_1)):
        lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            f"lid_{i} seats on the body rim at closed position",
            lid_aabb is not None and abs(lid_aabb[0][2] - BODY_H) < 0.005,
            details=f"lid_{i} aabb={lid_aabb}",
        )
        ctx.check(
            f"lid_{i} panel top is near ~0.37 m (body + lid thickness)",
            lid_aabb is not None and 0.36 < lid_aabb[1][2] < 0.42,
            details=f"lid_{i} aabb={lid_aabb}",
        )

    # --- both panels together span the coffin length -----------------------
    aabb_0 = ctx.part_world_aabb(lid_0)
    aabb_1 = ctx.part_world_aabb(lid_1)
    ctx.check(
        "lid_0 covers the head end (x < 0) and lid_1 covers the foot end (x > 0)",
        aabb_0 is not None
        and aabb_1 is not None
        and aabb_0[0][0] < -0.80
        and aabb_0[1][0] < 0.05
        and aabb_1[0][0] > -0.05
        and aabb_1[1][0] > 0.80,
        details=f"lid_0 x=({aabb_0[0][0]:.3f}, {aabb_0[1][0]:.3f}), "
        f"lid_1 x=({aabb_1[0][0]:.3f}, {aabb_1[1][0]:.3f})",
    )

    # --- visible gap between the two panels (no overlap) --------------------
    ctx.expect_gap(lid_1, lid_0, axis="x", min_gap=0.0, max_gap=0.020, name="gap between lid panels at split line")

    # --- both hinges: revolute along the coffin length, 0..~110 deg ---------
    for i, hinge in enumerate((hinge_0, hinge_1)):
        limits = hinge.motion_limits
        ctx.check(
            f"lid_hinge_{i} is revolute with range 0 to ~110 degrees",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and abs(limits.lower) < 1e-9
            and abs(limits.upper - LID_OPEN_MAX) < 0.02,
            details=f"type={hinge.articulation_type}, limits=({limits.lower}, {limits.upper})",
        )
        ctx.check(
            f"lid_hinge_{i} axis runs along the coffin's length (x)",
            abs(abs(hinge.axis[0]) - 1.0) < 1e-6
            and abs(hinge.axis[1]) < 1e-6
            and abs(hinge.axis[2]) < 1e-6,
            details=f"axis={hinge.axis}",
        )

    # --- each panel opens independently --------------------------------------
    with ctx.pose({hinge_0: LID_OPEN_MAX, hinge_1: 0.0}):
        open_0 = ctx.part_world_aabb(lid_0)
        closed_1 = ctx.part_world_aabb(lid_1)
        ctx.check(
            "head panel (lid_0) opens past vertical while foot panel stays closed",
            open_0 is not None
            and closed_1 is not None
            and open_0[1][2] > 0.70
            and closed_1[1][2] < 0.42,
            details=f"open lid_0={open_0}, closed lid_1={closed_1}",
        )

    with ctx.pose({hinge_0: 0.0, hinge_1: LID_OPEN_MAX}):
        closed_0 = ctx.part_world_aabb(lid_0)
        open_1 = ctx.part_world_aabb(lid_1)
        ctx.check(
            "foot panel (lid_1) opens past vertical while head panel stays closed",
            closed_0 is not None
            and open_1 is not None
            and closed_0[1][2] < 0.42
            and open_1[1][2] > 0.70,
            details=f"closed lid_0={closed_0}, open lid_1={open_1}",
        )

    with ctx.pose({hinge_0: LID_OPEN_MAX, hinge_1: LID_OPEN_MAX}):
        both_open_0 = ctx.part_world_aabb(lid_0)
        both_open_1 = ctx.part_world_aabb(lid_1)
        ctx.check(
            "both panels open simultaneously without blocking each other",
            both_open_0 is not None
            and both_open_1 is not None
            and both_open_0[1][2] > 0.70
            and both_open_1[1][2] > 0.70,
            details=f"lid_0={both_open_0}, lid_1={both_open_1}",
        )

    # --- cross sits proud on the head-end panel (lid_0) --------------------
    cross_closed = ctx.part_element_world_aabb(lid_0, elem="cross_upright")
    ctx.check(
        "Latin cross sits proud on the head-end lid panel (lid_0)",
        cross_closed is not None
        and cross_closed[1][2] > BODY_H + LID_T + 0.005
        and cross_closed[1][0] < SHOULDER_X
        and abs((cross_closed[0][1] + cross_closed[1][1]) / 2.0) < 0.02,
        details=f"cross aabb={cross_closed}",
    )
    cross_arm = ctx.part_element_world_aabb(lid_0, elem="cross_arm")
    ctx.check(
        "cross arm spans wider than the upright near the head end",
        cross_arm is not None
        and cross_closed is not None
        and (cross_arm[1][1] - cross_arm[0][1]) > 0.15
        and cross_arm[0][0] < cross_closed[0][0] + 0.15,
        details=f"arm aabb={cross_arm}",
    )

    # Cross rides with head panel through hinge rotation.
    with ctx.pose({hinge_0: LID_OPEN_MAX}):
        cross_open = ctx.part_element_world_aabb(lid_0, elem="cross_upright")
        ctx.check(
            "cross rides with the head panel (lid_0) through the hinge rotation",
            cross_closed is not None
            and cross_open is not None
            and cross_open[0][2] > cross_closed[1][2] + 0.10,
            details=f"closed={cross_closed}, open={cross_open}",
        )

    # --- strap boards on correct lid panels ---------------------------------
    # lid_0 carries head_strap and shoulder_strap; lid_1 carries foot_strap.
    lid_0_strap_count = len(LID_PANEL_STRAPS[0])
    lid_1_strap_count = len(LID_PANEL_STRAPS[1])
    ctx.check(
        f"lid_0 carries {lid_0_strap_count} strap boards (head + shoulder)",
        lid_0_strap_count == 2,
        details=f"straps on lid_0: {LID_PANEL_STRAPS[0]}",
    )
    ctx.check(
        f"lid_1 carries {lid_1_strap_count} strap board (foot)",
        lid_1_strap_count == 1,
        details=f"straps on lid_1: {LID_PANEL_STRAPS[1]}",
    )

    # Verify strap visuals exist on each panel.
    for i, lid in enumerate((lid_0, lid_1)):
        for j in range(len(LID_PANEL_STRAPS[i])):
            strap_aabb = ctx.part_element_world_aabb(lid, elem=f"lid_strap_{i}_{j}")
            ctx.check(
                f"lid_{i} strap_{j} exists and sits above the body rim",
                strap_aabb is not None and strap_aabb[0][2] > BODY_H - 0.01,
                details=f"strap aabb={strap_aabb}",
            )

    # --- three strap boards wrap the body sides (unchanged) -----------------
    for strap_name, x_center in STRAP_XS.items():
        body_band = ctx.part_element_world_aabb(body, elem=f"body_{strap_name}")
        ctx.check(
            f"{strap_name} board wraps down both body side walls",
            body_band is not None
            and abs((body_band[0][0] + body_band[1][0]) / 2.0 - x_center) < 0.02
            and (body_band[1][2] - body_band[0][2]) > 0.25
            and (body_band[1][1] - body_band[0][1]) > 0.40,
            details=f"body band aabb={body_band}",
        )

    # --- toe-pincher taper: shoulder widest ---------------------------------
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "shoulder width exceeds head and foot widths (toe-pincher taper)",
        shell_aabb is not None
        and 2.0 * SHOULDER_HALF_W > 2.0 * HEAD_HALF_W > 2.0 * FOOT_HALF_W
        and abs((shell_aabb[1][1] - shell_aabb[0][1]) - 2.0 * SHOULDER_HALF_W) < 0.01,
        details=f"shell aabb={shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
