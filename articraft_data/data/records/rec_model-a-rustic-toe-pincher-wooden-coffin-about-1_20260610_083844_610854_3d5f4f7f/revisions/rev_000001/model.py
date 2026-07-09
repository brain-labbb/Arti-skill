from __future__ import annotations

"""Rustic toe-pincher wooden coffin with a side-hinged lid.

A hollow six-sided (toe-pincher) plank box about 1.9 m long, 0.6 m wide at the
shoulders, tapering to ~0.35 m at the head and ~0.3 m at the foot, ~0.4 m tall
overall. Three darker strap boards wrap across the lid and down the body sides
(near the head, at the shoulder break, near the foot), and a dark Latin cross
sits on the upper lid panel. The flat hexagonal lid hinges along one long side
edge at the top rim and swings open 0..110 degrees to reveal the hollow
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

HINGE_Y = -(SHOULDER_HALF_W + LID_OVERHANG)  # hinge line along the -Y rim edge
LID_OPEN_MAX = math.radians(110.0)

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


def _lid_panel():
    """Flat hexagonal lid plank panel with lengthwise plank seam grooves."""
    panel = _extrude_poly(LID_POLY, 0.0, LID_T)
    # Three lengthwise planks: two shallow seam grooves across the top face.
    lid_w = 2.0 * (SHOULDER_HALF_W + LID_OVERHANG)
    for y_seam in (lid_w / 3.0, 2.0 * lid_w / 3.0):
        groove = cq.Workplane("XY", origin=(0.0, y_seam, LID_T)).box(2.2, 0.004, 0.008)
        panel = panel.cut(groove)
    return panel


def _lid_strap(x_center):
    """Darker strap board lying across the lid and wrapping over its edges."""
    cap = _extrude_poly(_offset_polygon(LID_POLY, STRAP_T), 0.0, LID_T + STRAP_T).cut(
        _extrude_poly(_offset_polygon(LID_POLY, -0.002), -0.01, LID_T - STRAP_EMBED)
    )
    return cap.intersect(_x_slab(x_center, STRAP_W, z_center=0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toe_pincher_coffin")

    plank_wood = model.material("plank_wood", rgba=(0.44, 0.31, 0.20, 1.0))
    strap_wood = model.material("strap_wood", rgba=(0.29, 0.19, 0.12, 1.0))
    cross_wood = model.material("cross_wood", rgba=(0.22, 0.14, 0.09, 1.0))

    # ---- hollow box body -----------------------------------------------
    body = model.part("coffin_body")
    shell = _body_shell()
    body.visual(mesh_from_cadquery(shell, "body_shell"), material=plank_wood, name="body_shell")
    for strap_name, x_center in STRAP_XS.items():
        body.visual(
            mesh_from_cadquery(_body_strap_band(x_center), f"body_{strap_name}"),
            material=strap_wood,
            name=f"body_{strap_name}",
        )

    # ---- hinged lid ------------------------------------------------------
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_panel(), "lid_panel"), material=plank_wood, name="lid_panel")
    for strap_name, x_center in STRAP_XS.items():
        lid.visual(
            mesh_from_cadquery(_lid_strap(x_center), f"lid_{strap_name}"),
            material=strap_wood,
            name=f"lid_{strap_name}",
        )

    # Latin cross on the upper (head-end) lid panel, top toward the head.
    lid_center_y = SHOULDER_HALF_W + LID_OVERHANG  # lid local y of the centerline
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
    # Closed lid extends along local +Y, so positive q about +X lifts the
    # free edge upward (right-hand rule) and swings it open toward -Y.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=0.0, upper=LID_OPEN_MAX),
    )

    # Record hollowness evidence for prompt-specific tests.
    outer_volume = float(_extrude_poly(FOOTPRINT, 0.0, BODY_H).val().Volume())
    shell_volume = float(shell.val().Volume())
    model.meta["outer_solid_volume"] = outer_volume
    model.meta["shell_volume"] = shell_volume

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("coffin_body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

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
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "closed coffin is ~0.4 m tall overall",
        lid_aabb is not None and 0.36 < lid_aabb[1][2] < 0.42,
        details=f"lid aabb={lid_aabb}",
    )

    # Toe-pincher taper: head end wider than foot end, shoulders widest.
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

    # --- closed lid seats on the rim and overhangs it ------------------------
    ctx.expect_gap(lid, body, axis="z", max_gap=0.002, max_penetration=0.0005)
    ctx.expect_contact(lid, body, contact_tol=1e-4)
    ctx.expect_within(
        body,
        lid,
        axes="xy",
        inner_elem="body_shell",
        outer_elem="lid_panel",
        margin=0.001,
        name="lid outline overhangs the box rim on all sides",
    )

    # --- hinge: revolute along the coffin's length, 0..~110 deg -------------
    limits = hinge.motion_limits
    ctx.check(
        "lid hinge is revolute with range 0 to ~110 degrees",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - LID_OPEN_MAX) < 0.02,
        details=f"type={hinge.articulation_type}, limits=({limits.lower}, {limits.upper})",
    )
    ctx.check(
        "hinge axis runs along the coffin's length (x)",
        abs(abs(hinge.axis[0]) - 1.0) < 1e-6 and abs(hinge.axis[1]) < 1e-6 and abs(hinge.axis[2]) < 1e-6,
        details=f"axis={hinge.axis}",
    )

    cross_closed = ctx.part_element_world_aabb(lid, elem="cross_upright")
    with ctx.pose({hinge: LID_OPEN_MAX}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "open lid swings up past vertical, revealing the interior",
            open_lid_aabb is not None and open_lid_aabb[1][2] > 0.80,
            details=f"open lid aabb={open_lid_aabb}",
        )
        ctx.check(
            "open lid clears the box opening toward the hinge side",
            open_lid_aabb is not None and open_lid_aabb[1][1] < -0.25,
            details=f"open lid aabb={open_lid_aabb}",
        )
        cross_open = ctx.part_element_world_aabb(lid, elem="cross_upright")
        ctx.check(
            "off-axis cross rides with the lid through the hinge rotation",
            cross_closed is not None
            and cross_open is not None
            and cross_open[0][2] > cross_closed[1][2] + 0.10,
            details=f"closed={cross_closed}, open={cross_open}",
        )

    # --- cross sits proud on the upper (head-end) lid panel ------------------
    ctx.check(
        "Latin cross sits proud on the head-end lid panel",
        cross_closed is not None
        and cross_closed[1][2] > BODY_H + LID_T + 0.005
        and cross_closed[1][0] < SHOULDER_X
        and abs((cross_closed[0][1] + cross_closed[1][1]) / 2.0) < 0.02,
        details=f"cross aabb={cross_closed}",
    )
    cross_arm = ctx.part_element_world_aabb(lid, elem="cross_arm")
    ctx.check(
        "cross arm spans wider than the upright near the head end",
        cross_arm is not None
        and cross_closed is not None
        and (cross_arm[1][1] - cross_arm[0][1]) > 0.15
        and cross_arm[0][0] < cross_closed[0][0] + 0.15,
        details=f"arm aabb={cross_arm}",
    )

    # --- three strap boards wrap the lid and body sides ----------------------
    for strap_name, x_center in STRAP_XS.items():
        lid_strap = ctx.part_element_world_aabb(lid, elem=f"lid_{strap_name}")
        ctx.check(
            f"{strap_name} board crosses the full lid width at x~{x_center:+.2f}",
            lid_strap is not None
            and abs((lid_strap[0][0] + lid_strap[1][0]) / 2.0 - x_center) < 0.02
            and (lid_strap[1][1] - lid_strap[0][1]) > 0.28
            and lid_strap[1][2] > BODY_H + LID_T + 0.005,
            details=f"lid strap aabb={lid_strap}",
        )
        body_band = ctx.part_element_world_aabb(body, elem=f"body_{strap_name}")
        ctx.check(
            f"{strap_name} board wraps down both body side walls",
            body_band is not None
            and abs((body_band[0][0] + body_band[1][0]) / 2.0 - x_center) < 0.02
            and (body_band[1][2] - body_band[0][2]) > 0.25
            and (body_band[1][1] - body_band[0][1]) > 0.40,
            details=f"body band aabb={body_band}",
        )

    return ctx.report()


object_model = build_object_model()
