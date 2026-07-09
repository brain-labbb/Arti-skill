from __future__ import annotations

"""Half-couch toe-pincher wooden coffin.

Variant of the rustic toe-pincher coffin where only the head-half of the lid
hinges open on the -Y side rim while the foot-half stays fixed and closed —
a half-couch casket configuration.  Body shell, three strap bands, and the
dark Latin cross are carried over from the parent.
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

# ---------------------------------------------------------------------------
# Dimensions (m).  Coffin along +X, head at -X, foot at +X, z up.
# ---------------------------------------------------------------------------
LENGTH = 1.90
HEAD_HALF_W = 0.175
SHOULDER_HALF_W = 0.30
FOOT_HALF_W = 0.15
HEAD_X = -LENGTH / 2.0
FOOT_X = LENGTH / 2.0
SHOULDER_X = HEAD_X + LENGTH / 3.0

WALL_T = 0.03
FLOOR_T = 0.03
BODY_H = 0.33
LID_T = 0.04
LID_OVERHANG = 0.015
STRAP_T = 0.012
STRAP_W = 0.08
STRAP_EMBED = 0.003

HINGE_Y = -(SHOULDER_HALF_W + LID_OVERHANG)
LID_OPEN_MAX = math.radians(110.0)

# Half-couch split: head-half lid opens, foot-half stays closed.
SPLIT_X = 0.0
SEAM_GAP = 0.004  # visible seam between the two lid halves

# Three strap stations, emitted via indexed loops.
N_STRAPS = 3
STRAP_XS = (-0.75, SHOULDER_X, 0.65)

# Toe-pincher footprint (CCW).
FOOTPRINT = (
    (HEAD_X, -HEAD_HALF_W),
    (SHOULDER_X, -SHOULDER_HALF_W),
    (FOOT_X, -FOOT_HALF_W),
    (FOOT_X, FOOT_HALF_W),
    (SHOULDER_X, SHOULDER_HALF_W),
    (HEAD_X, HEAD_HALF_W),
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _offset_polygon(pts, dist):
    """Offset a convex CCW polygon: positive dist expands outward."""
    n = len(pts)
    lines = []
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        dx, dy = dx / length, dy / length
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
    """Y-wide slab for cropping wrap bands to one strap station."""
    return cq.Workplane("XY", origin=(x_center, 0.0, z_center)).box(
        width, 2.0, z_height
    )


def _clip_x(shape, x_min, x_max, z_center, z_height=1.0):
    """Clip a shape to an x range with a wide y/z slab."""
    x_mid = (x_min + x_max) / 2.0
    slab = cq.Workplane("XY", origin=(x_mid, 0.0, z_center)).box(
        x_max - x_min, 3.0, z_height
    )
    return shape.intersect(slab)


# Derived polygons (module-level so helpers can reference them).
WORLD_LID_POLY = _offset_polygon(FOOTPRINT, LID_OVERHANG)

# Lid-local polygon: origin on the hinge line, panel extends along local +Y.
LID_POLY = tuple((x, y - HINGE_Y) for (x, y) in WORLD_LID_POLY)


def _body_shell():
    """Hollow toe-pincher box: open top, 0.03 m walls and floor, plank seams."""
    outer = _extrude_poly(FOOTPRINT, 0.0, BODY_H)
    cavity = _extrude_poly(
        _offset_polygon(FOOTPRINT, -WALL_T), FLOOR_T, BODY_H + 0.01
    )
    shell = outer.cut(cavity)
    for z_seam in (0.11, 0.22):
        ring = _extrude_poly(
            FOOTPRINT, z_seam - 0.0025, z_seam + 0.0025
        ).cut(
            _extrude_poly(
                _offset_polygon(FOOTPRINT, -0.005),
                z_seam - 0.004,
                z_seam + 0.004,
            )
        )
        shell = shell.cut(ring)
    return shell


def _body_strap_band(x_center):
    """Darker strap board wrapping the body side walls at one station."""
    ring = _extrude_poly(
        _offset_polygon(FOOTPRINT, STRAP_T), 0.02, BODY_H
    ).cut(
        _extrude_poly(
            _offset_polygon(FOOTPRINT, -0.002), 0.015, BODY_H + 0.01
        )
    )
    return ring.intersect(_x_slab(x_center, STRAP_W))


# -- head lid (lid-local coordinates, hinge at y=0 z=0) -------------------

def _head_lid_panel():
    """Head-half lid plank panel in lid-local coords."""
    panel = _extrude_poly(LID_POLY, 0.0, LID_T)
    panel = _clip_x(
        panel, HEAD_X, SPLIT_X - SEAM_GAP / 2,
        z_center=LID_T / 2, z_height=LID_T + 0.02,
    )
    lid_w = 2.0 * (SHOULDER_HALF_W + LID_OVERHANG)
    for y_seam in (lid_w / 3.0, 2.0 * lid_w / 3.0):
        groove = (
            cq.Workplane("XY", origin=(0.0, y_seam, LID_T))
            .box(2.2, 0.004, 0.008)
        )
        panel = panel.cut(groove)
    return panel


def _lid_strap_local(x_center):
    """Strap board on the head lid, in lid-local coords."""
    cap = _extrude_poly(
        _offset_polygon(LID_POLY, STRAP_T), 0.0, LID_T + STRAP_T
    ).cut(
        _extrude_poly(
            _offset_polygon(LID_POLY, -0.002), -0.01, LID_T - STRAP_EMBED
        )
    )
    return cap.intersect(_x_slab(x_center, STRAP_W, z_center=0.0))


# -- foot lid (world/body coordinates, sits on the body rim) ---------------

def _foot_lid_panel():
    """Foot-half lid plank panel in world/body coords."""
    panel = _extrude_poly(WORLD_LID_POLY, BODY_H, BODY_H + LID_T)
    panel = _clip_x(
        panel, SPLIT_X + SEAM_GAP / 2, FOOT_X,
        z_center=BODY_H + LID_T / 2, z_height=LID_T + 0.02,
    )
    lid_w = 2.0 * (SHOULDER_HALF_W + LID_OVERHANG)
    for y_frac in (1.0 / 3.0, 2.0 / 3.0):
        y_seam = HINGE_Y + y_frac * lid_w
        groove = (
            cq.Workplane("XY", origin=(0.0, y_seam, BODY_H + LID_T))
            .box(2.2, 0.004, 0.008)
        )
        panel = panel.cut(groove)
    return panel


def _foot_lid_strap(x_center):
    """Strap board on the foot lid, in world/body coords."""
    cap = _extrude_poly(
        _offset_polygon(WORLD_LID_POLY, STRAP_T),
        BODY_H,
        BODY_H + LID_T + STRAP_T,
    ).cut(
        _extrude_poly(
            _offset_polygon(WORLD_LID_POLY, -0.002),
            BODY_H - 0.01,
            BODY_H + LID_T - STRAP_EMBED,
        )
    )
    return cap.intersect(_x_slab(x_center, STRAP_W, z_center=BODY_H))


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_couch_coffin")

    plank_wood = model.material("plank_wood", rgba=(0.44, 0.31, 0.20, 1.0))
    strap_wood = model.material("strap_wood", rgba=(0.29, 0.19, 0.12, 1.0))
    cross_wood = model.material("cross_wood", rgba=(0.22, 0.14, 0.09, 1.0))

    # ---- coffin body (root) ------------------------------------------------
    body = model.part("coffin_body")
    shell = _body_shell()
    body.visual(
        mesh_from_cadquery(shell, "body_shell"),
        material=plank_wood,
        name="body_shell",
    )

    # Body-wrap strap boards (indexed loop).
    for i in range(N_STRAPS):
        body.visual(
            mesh_from_cadquery(_body_strap_band(STRAP_XS[i]), f"body_strap_{i}"),
            material=strap_wood,
            name=f"body_strap_{i}",
        )

    # Foot-half lid panel and straps are fixed to the body (inlined visuals).
    body.visual(
        mesh_from_cadquery(_foot_lid_panel(), "foot_lid_panel"),
        material=plank_wood,
        name="foot_lid_panel",
    )
    for i in range(N_STRAPS):
        if STRAP_XS[i] >= SPLIT_X:
            body.visual(
                mesh_from_cadquery(
                    _foot_lid_strap(STRAP_XS[i]), f"foot_lid_strap_{i}"
                ),
                material=strap_wood,
                name=f"foot_lid_strap_{i}",
            )

    # ---- head-half lid (hinged) -------------------------------------------
    head_lid = model.part("head_lid")
    head_lid.visual(
        mesh_from_cadquery(_head_lid_panel(), "head_lid_panel"),
        material=plank_wood,
        name="head_lid_panel",
    )
    for i in range(N_STRAPS):
        if STRAP_XS[i] < SPLIT_X:
            head_lid.visual(
                mesh_from_cadquery(
                    _lid_strap_local(STRAP_XS[i]), f"head_lid_strap_{i}"
                ),
                material=strap_wood,
                name=f"head_lid_strap_{i}",
            )

    # Latin cross on the upper (head-end) lid panel.
    lid_center_y = SHOULDER_HALF_W + LID_OVERHANG
    cross_z = LID_T - STRAP_EMBED + 0.0075
    head_lid.visual(
        Box((0.32, 0.05, 0.015)),
        origin=Origin(xyz=(-0.54, lid_center_y, cross_z)),
        material=cross_wood,
        name="cross_upright",
    )
    head_lid.visual(
        Box((0.05, 0.20, 0.015)),
        origin=Origin(xyz=(-0.61, lid_center_y, cross_z)),
        material=cross_wood,
        name="cross_arm",
    )

    # Hinge along the coffin length at the top rim of the -Y side wall.
    # Positive q about +X lifts the free edge upward (right-hand rule).
    model.articulation(
        "head_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head_lid,
        origin=Origin(xyz=(0.0, HINGE_Y, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=1.0, lower=0.0, upper=LID_OPEN_MAX
        ),
    )

    # Hollowness evidence for prompt-specific tests.
    outer_volume = float(_extrude_poly(FOOTPRINT, 0.0, BODY_H).val().Volume())
    shell_volume = float(shell.val().Volume())
    model.meta["outer_solid_volume"] = outer_volume
    model.meta["shell_volume"] = shell_volume

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("coffin_body")
    head_lid = object_model.get_part("head_lid")
    hinge = object_model.get_articulation("head_lid_hinge")

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
        "box sits on the ground (z_min ~ 0)",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.005,
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

    # --- half-couch split: head lid in head half, foot lid in foot half -----
    head_panel_aabb = ctx.part_element_world_aabb(head_lid, elem="head_lid_panel")
    ctx.check(
        "head lid panel is in the head half (x <= split)",
        head_panel_aabb is not None and head_panel_aabb[1][0] <= SPLIT_X + 0.01,
        details=f"head_lid_panel aabb={head_panel_aabb}",
    )

    foot_panel_aabb = ctx.part_element_world_aabb(body, elem="foot_lid_panel")
    ctx.check(
        "foot lid panel is in the foot half (x >= split) at rim height",
        foot_panel_aabb is not None
        and foot_panel_aabb[0][0] >= SPLIT_X - 0.01
        and abs(foot_panel_aabb[0][2] - BODY_H) < 0.005
        and abs(foot_panel_aabb[1][2] - (BODY_H + LID_T)) < 0.005,
        details=f"foot_lid_panel aabb={foot_panel_aabb}",
    )

    # Visible seam gap between the two lid halves at the split line.
    ctx.expect_gap(
        body, head_lid, axis="x",
        positive_elem="foot_lid_panel",
        negative_elem="head_lid_panel",
        min_gap=0.001,
        max_gap=0.01,
        name="visible seam gap at half-couch split",
    )

    # --- closed head lid seats on the rim -----------------------------------
    ctx.expect_gap(
        head_lid, body, axis="z",
        positive_elem="head_lid_panel",
        negative_elem="body_shell",
        max_gap=0.002, max_penetration=0.0005,
        name="head lid seats on body rim when closed",
    )
    ctx.expect_contact(
        head_lid, body, contact_tol=1e-4,
        elem_a="head_lid_panel", elem_b="body_shell",
        name="head lid contacts body when closed",
    )

    # --- hinge: revolute along coffin length, 0..~110 deg ------------------
    limits = hinge.motion_limits
    ctx.check(
        "head lid hinge is revolute with range 0 to ~110 degrees",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - LID_OPEN_MAX) < 0.02,
        details=f"type={hinge.articulation_type}, limits=({limits.lower}, {limits.upper})",
    )
    ctx.check(
        "hinge axis runs along the coffin's length (x)",
        abs(abs(hinge.axis[0]) - 1.0) < 1e-6
        and abs(hinge.axis[1]) < 1e-6
        and abs(hinge.axis[2]) < 1e-6,
        details=f"axis={hinge.axis}",
    )

    # --- open pose: only head lid moves, foot lid stays closed --------------
    cross_closed = ctx.part_element_world_aabb(head_lid, elem="cross_upright")
    foot_closed_z = foot_panel_aabb[1][2] if foot_panel_aabb else None

    with ctx.pose({hinge: LID_OPEN_MAX}):
        open_lid_aabb = ctx.part_world_aabb(head_lid)
        ctx.check(
            "open head lid swings up past vertical, revealing interior",
            open_lid_aabb is not None and open_lid_aabb[1][2] > 0.70,
            details=f"open head_lid aabb={open_lid_aabb}",
        )
        ctx.check(
            "open head lid clears the box opening toward the hinge side",
            open_lid_aabb is not None and open_lid_aabb[1][1] < -0.25,
            details=f"open head_lid aabb={open_lid_aabb}",
        )

        # Foot lid stays closed.
        foot_open = ctx.part_element_world_aabb(body, elem="foot_lid_panel")
        ctx.check(
            "foot lid stays closed when head lid opens (half-couch)",
            foot_open is not None
            and foot_closed_z is not None
            and abs(foot_open[1][2] - foot_closed_z) < 0.005,
            details=(
                f"foot_closed_z={foot_closed_z}, "
                f"foot_open_z={foot_open[1][2] if foot_open else None}"
            ),
        )

        # Cross rides with the head lid.
        cross_open = ctx.part_element_world_aabb(head_lid, elem="cross_upright")
        ctx.check(
            "cross rides with the head lid through the hinge rotation",
            cross_closed is not None
            and cross_open is not None
            and cross_open[0][2] > cross_closed[1][2] + 0.10,
            details=f"closed={cross_closed}, open={cross_open}",
        )

    # --- cross sits proud on the upper (head-end) lid panel -----------------
    ctx.check(
        "Latin cross sits proud on the head-end lid panel",
        cross_closed is not None
        and cross_closed[1][2] > BODY_H + LID_T + 0.005
        and cross_closed[1][0] < SHOULDER_X
        and abs((cross_closed[0][1] + cross_closed[1][1]) / 2.0) < 0.02,
        details=f"cross aabb={cross_closed}",
    )
    cross_arm = ctx.part_element_world_aabb(head_lid, elem="cross_arm")
    ctx.check(
        "cross arm spans wider than the upright near the head end",
        cross_arm is not None
        and cross_closed is not None
        and (cross_arm[1][1] - cross_arm[0][1]) > 0.15
        and cross_arm[0][0] < cross_closed[0][0] + 0.15,
        details=f"arm aabb={cross_arm}",
    )

    # --- three strap boards wrap the body sides -----------------------------
    for i in range(N_STRAPS):
        body_band = ctx.part_element_world_aabb(body, elem=f"body_strap_{i}")
        ctx.check(
            f"body_strap_{i} wraps body side walls at x~{STRAP_XS[i]:+.2f}",
            body_band is not None
            and abs((body_band[0][0] + body_band[1][0]) / 2.0 - STRAP_XS[i]) < 0.02
            and (body_band[1][2] - body_band[0][2]) > 0.25
            and (body_band[1][1] - body_band[0][1]) > 0.40,
            details=f"body_strap_{i} aabb={body_band}",
        )

    # Head-lid straps exist on the hinged half.
    for i in range(N_STRAPS):
        if STRAP_XS[i] < SPLIT_X:
            s = ctx.part_element_world_aabb(head_lid, elem=f"head_lid_strap_{i}")
            ctx.check(
                f"head_lid_strap_{i} crosses head lid at x~{STRAP_XS[i]:+.2f}",
                s is not None
                and abs((s[0][0] + s[1][0]) / 2.0 - STRAP_XS[i]) < 0.02,
                details=f"head_lid_strap_{i} aabb={s}",
            )

    # Foot-lid straps exist on the fixed half.
    for i in range(N_STRAPS):
        if STRAP_XS[i] >= SPLIT_X:
            s = ctx.part_element_world_aabb(body, elem=f"foot_lid_strap_{i}")
            ctx.check(
                f"foot_lid_strap_{i} crosses foot lid at x~{STRAP_XS[i]:+.2f}",
                s is not None
                and abs((s[0][0] + s[1][0]) / 2.0 - STRAP_XS[i]) < 0.02,
                details=f"foot_lid_strap_{i} aabb={s}",
            )

    return ctx.report()


object_model = build_object_model()
