from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Six-panel Chinese-style folding screen (accordion room divider).
#
# World frame: Z up, decorated front faces -Y.
# Panel 0 is the fixed root, centered at the world origin.
# Panels 1..5 are chained: panel_i hinges to panel_{i-1} at the
# vertical seam on panel_{i-1}'s outer edge.  The rest pose alternates
# fold direction to create the accordion zigzag (odd hinges fold
# forward, even hinges fold backward).
# ---------------------------------------------------------------------------

N_PANELS = 6
PANEL_W = 0.60
PANEL_H = 1.70
PANEL_T = 0.03
RAIL_W = 0.05
BOTTOM_RAIL_H = 0.25
TOP_RAIL_H = 0.05
SEAM_GAP = 0.013
FOLD_ANGLE = math.radians(35.0)
FOLD_RANGE = math.radians(150.0)

OPEN_W = PANEL_W - 2.0 * RAIL_W
OPEN_H = PANEL_H - BOTTOM_RAIL_H - TOP_RAIL_H
OPEN_ZC = BOTTOM_RAIL_H + OPEN_H / 2.0

FRET_BAR = 0.015
FRET_DEPTH = 0.008
HINGE_HEIGHTS = (0.35, 1.35)

# Hinge-x in the parent panel's local frame
ROOT_HINGE_X = PANEL_W / 2.0 + SEAM_GAP  # 0.313
CHAIN_HINGE_X = PANEL_W + 2.0 * SEAM_GAP  # 0.626


# ---- shared geometry helpers ------------------------------------------------

def _fret_lattice() -> cq.Workplane:
    """One panel's gold fretwork: interlocking rectangles and squares."""
    t = FRET_BAR
    d = FRET_DEPTH

    def ring(cx: float, cy: float, w: float, h: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .center(cx, cy)
            .rect(w, h)
            .rect(w - 2.0 * t, h - 2.0 * t)
            .extrude(d)
        )

    def bar(cx: float, cy: float, w: float, h: float) -> cq.Workplane:
        return cq.Workplane("XY").center(cx, cy).rect(w, h).extrude(d)

    solid = ring(0.0, 0.0, 0.49, 1.38)
    for vc in (-0.43, 0.0, 0.43):
        solid = solid.union(ring(0.0, vc, 0.36, 0.32))
        solid = solid.union(ring(0.0, vc, 0.20, 0.16))
        for s in (1.0, -1.0):
            solid = solid.union(bar(s * 0.14, vc, 0.09, t))
            solid = solid.union(bar(0.0, vc + s * 0.12, t, 0.09))
            solid = solid.union(bar(s * 0.205, vc, 0.06, t))
            for s2 in (1.0, -1.0):
                solid = solid.union(ring(s * 0.18, vc + s2 * 0.16, 0.09, 0.09))
    for vc in (-0.215, 0.215):
        solid = solid.union(bar(0.0, vc, t, 0.13))
    for vc in (-0.635, 0.635):
        solid = solid.union(bar(0.0, vc, t, 0.10))
    return solid


def _add_panel(part, x0: float, x1: float, dz: float, mats: dict, fret_mesh) -> None:
    """Author one lacquered screen panel spanning part-local x in [x0, x1].

    dz shifts nominal floor-based heights into the part frame (chained
    part frames sit on the lower hinge barrel at nominal z = 0.35).
    """
    cx = (x0 + x1) / 2.0
    w = x1 - x0

    def z(v: float) -> float:
        return v - dz

    lacquer = mats["lacquer"]
    worn = mats["worn_lacquer"]
    black = mats["black_field"]
    gold = mats["gold"]

    # Frame: solid bottom rail, top rail, two side stiles.
    part.visual(
        Box((w, PANEL_T, BOTTOM_RAIL_H)),
        origin=Origin(xyz=(cx, 0.0, z(BOTTOM_RAIL_H / 2.0))),
        material=worn,
        name="bottom_rail",
    )
    part.visual(
        Box((w, PANEL_T, TOP_RAIL_H)),
        origin=Origin(xyz=(cx, 0.0, z(PANEL_H - TOP_RAIL_H / 2.0))),
        material=lacquer,
        name="top_rail",
    )
    for idx, sx in enumerate((x0 + RAIL_W / 2.0, x1 - RAIL_W / 2.0)):
        part.visual(
            Box((RAIL_W, PANEL_T, OPEN_H + 0.02)),
            origin=Origin(xyz=(sx, 0.0, z(OPEN_ZC))),
            material=lacquer,
            name=f"stile_{idx}",
        )
    # Matte black inset field.
    part.visual(
        Box((OPEN_W + 0.01, 0.012, OPEN_H + 0.01)),
        origin=Origin(xyz=(cx, 0.0, z(OPEN_ZC))),
        material=black,
        name="lattice_field",
    )
    # Gold fretwork, raised proud of the front face.
    part.visual(
        fret_mesh,
        origin=Origin(xyz=(cx, -0.003, z(OPEN_ZC)), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="fretwork",
    )


def _add_barrel_hardware(part, dz: float, mats: dict) -> None:
    """Child-side hinge barrels, webs, and leaf plates at x=0 in part frame."""
    brass = mats["brass"]
    for idx, zh in enumerate(HINGE_HEIGHTS):
        zl = zh - dz
        part.visual(
            Cylinder(radius=0.012, length=0.10),
            origin=Origin(xyz=(0.0, 0.0, zl)),
            material=brass,
            name=f"hinge_barrel_{idx}",
        )
        part.visual(
            Box((0.024, 0.022, 0.05)),
            origin=Origin(xyz=(0.012, 0.0, zl)),
            material=brass,
            name=f"hinge_web_{idx}",
        )
        part.visual(
            Box((0.05, 0.005, 0.04)),
            origin=Origin(xyz=(0.025, -0.017, zl)),
            material=brass,
            name=f"hinge_leaf_{idx}",
        )


def _add_knuckle_hardware(part, hinge_x: float, dz: float, mats: dict) -> None:
    """Parent-side hinge knuckles and leaf plate at hinge_x in part frame."""
    brass = mats["brass"]
    for idx, zh in enumerate(HINGE_HEIGHTS):
        zl = zh - dz
        part.visual(
            Box((0.036, 0.005, 0.07)),
            origin=Origin(xyz=(hinge_x - 0.033, -0.017, zl)),
            material=brass,
            name=f"knuckle_leaf_{idx}",
        )
        for k, sz in enumerate((-1.0, 1.0)):
            part.visual(
                Box((0.027, 0.022, 0.025)),
                origin=Origin(xyz=(hinge_x - 0.0075, 0.0, zl + sz * 0.035)),
                material=brass,
                name=f"knuckle_{idx}_{k}",
            )


# ---- build ------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="six_panel_folding_screen")

    mats = {
        "lacquer": model.material("red_brown_lacquer", rgba=(0.38, 0.10, 0.08, 1.0)),
        "worn_lacquer": model.material("worn_red_lacquer", rgba=(0.45, 0.15, 0.11, 1.0)),
        "black_field": model.material("matte_black_field", rgba=(0.05, 0.05, 0.06, 1.0)),
        "gold": model.material("antique_gold", rgba=(0.78, 0.62, 0.22, 1.0)),
        "brass": model.material("aged_brass", rgba=(0.55, 0.42, 0.18, 1.0)),
    }

    fret_mesh = mesh_from_cadquery(_fret_lattice(), "fret_lattice")

    panels = []
    for i in range(N_PANELS):
        panel = model.part(f"panel_{i}")

        if i == 0:
            # Root panel: centered at world origin, standing on the floor.
            _add_panel(panel, -PANEL_W / 2.0, PANEL_W / 2.0, 0.0, mats, fret_mesh)
            _add_knuckle_hardware(panel, ROOT_HINGE_X, 0.0, mats)
        else:
            # Chained panel: part frame sits on the inner hinge axis.
            _add_panel(panel, SEAM_GAP, SEAM_GAP + PANEL_W, HINGE_HEIGHTS[0], mats, fret_mesh)
            _add_barrel_hardware(panel, HINGE_HEIGHTS[0], mats)
            if i < N_PANELS - 1:
                _add_knuckle_hardware(panel, CHAIN_HINGE_X, HINGE_HEIGHTS[0], mats)

        panels.append(panel)

    # Vertical-axis hinges chaining each panel to the previous one.
    for i in range(1, N_PANELS):
        if i == 1:
            origin_x = ROOT_HINGE_X
            origin_z = HINGE_HEIGHTS[0]
        else:
            origin_x = CHAIN_HINGE_X
            origin_z = 0.0  # in parent's local frame (dz already shifts)

        # Alternating fold direction for the accordion zigzag.
        if i % 2 == 1:
            # Odd hinges fold forward (-Y): baked -FOLD_ANGLE.
            baked = -FOLD_ANGLE
            lower = -(FOLD_RANGE + FOLD_ANGLE)
            upper = FOLD_RANGE - FOLD_ANGLE
        else:
            # Even hinges fold backward (+Y): baked +FOLD_ANGLE.
            baked = FOLD_ANGLE
            lower = -(FOLD_RANGE - FOLD_ANGLE)
            upper = FOLD_RANGE + FOLD_ANGLE

        model.articulation(
            f"hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=panels[i - 1],
            child=panels[i],
            origin=Origin(xyz=(origin_x, 0.0, origin_z), rpy=(0.0, 0.0, baked)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=lower, upper=upper),
        )

    return model


# ---- tests ------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    hinges = [object_model.get_articulation(f"hinge_{i}") for i in range(1, N_PANELS)]

    # --- Hero structure: six full-height panels standing on the floor ---
    for panel in panels:
        aabb = ctx.part_world_aabb(panel)
        ctx.check(
            f"{panel.name} stands on the floor at full height",
            aabb is not None
            and -0.002 <= aabb[0][2] <= 0.01
            and 1.68 <= aabb[1][2] <= 1.72,
            details=f"aabb={aabb}",
        )

    # Five hinges connect the accordion chain.
    ctx.check(
        "five hinges connect the accordion chain",
        len(hinges) == 5,
        details=f"found {len(hinges)} hinges",
    )

    # Rest pose: root panel stays a thin upright slab.
    aabbs = [ctx.part_world_aabb(p) for p in panels]
    ctx.check(
        "root panel stays a thin upright slab",
        aabbs[0] is not None and aabbs[0][0][1] > -0.05 and aabbs[0][1][1] < 0.05,
        details=f"root={aabbs[0]}",
    )

    # Odd panels (1, 3, 5) angle forward (-Y) in the rest pose.
    for i in (1, 3, 5):
        ctx.check(
            f"panel_{i} angles forward in the rest pose",
            aabbs[i] is not None and aabbs[i][0][1] < -0.10,
            details=f"aabb_min_y={aabbs[i][0][1] if aabbs[i] else None}",
        )

    # Screen spans a reasonable total width.
    all_x_min = min(a[0][0] for a in aabbs if a is not None)
    all_x_max = max(a[1][0] for a in aabbs if a is not None)
    span = all_x_max - all_x_min
    ctx.check(
        "accordion screen spans at least 2.0 m width",
        span >= 2.0,
        details=f"span={span:.3f}",
    )

    # --- Hinge hardware: barrel/knuckle fit at each seam ---
    for i in range(1, N_PANELS):
        child = panels[i]
        parent = panels[i - 1]
        for idx in range(len(HINGE_HEIGHTS)):
            for k in range(2):
                ctx.allow_overlap(
                    child,
                    parent,
                    elem_a=f"hinge_barrel_{idx}",
                    elem_b=f"knuckle_{idx}_{k}",
                    reason="Hinge knuckle captures the barrel around the shared pin axis.",
                )
            ctx.expect_contact(
                child,
                parent,
                elem_a=f"hinge_barrel_{idx}",
                elem_b=f"knuckle_{idx}_0",
                name=f"panel_{i} barrel {idx} seated in panel_{i-1} knuckle",
            )

    # --- Gold fretwork present on every panel ---
    for panel in panels:
        fret = ctx.part_element_world_aabb(panel, elem="fretwork")
        ctx.check(
            f"{panel.name} carries gold fretwork across the opening height",
            fret is not None and fret[0][2] >= 0.15 and fret[1][2] <= 1.72,
            details=f"fret={fret}",
        )

    # Fretwork framed inside the black field on the root panel.
    field_0 = ctx.part_element_world_aabb(panels[0], elem="lattice_field")
    fret_0 = ctx.part_element_world_aabb(panels[0], elem="fretwork")
    ctx.check(
        "root fretwork stays framed inside the black field (x/z)",
        field_0 is not None
        and fret_0 is not None
        and fret_0[0][0] >= field_0[0][0] - 0.001
        and fret_0[1][0] <= field_0[1][0] + 0.001
        and fret_0[0][2] >= field_0[0][2] - 0.001
        and fret_0[1][2] <= field_0[1][2] + 0.001,
        details=f"field={field_0}, fret={fret_0}",
    )
    ctx.check(
        "root fretwork bars are raised proud of the black field",
        field_0 is not None
        and fret_0 is not None
        and fret_0[0][1] < field_0[0][1] - 0.003,
        details=f"field_front={field_0[0][1] if field_0 else None}, "
        f"fret_front={fret_0[0][1] if fret_0 else None}",
    )

    # Solid bottom rail is ~0.25 m tall.
    rail_0 = ctx.part_element_world_aabb(panels[0], elem="bottom_rail")
    ctx.check(
        "solid bottom rail is about 0.25 m tall from the floor",
        rail_0 is not None
        and rail_0[0][2] <= 0.002
        and 0.24 <= (rail_0[1][2] - rail_0[0][2]) <= 0.26,
        details=f"bottom_rail={rail_0}",
    )

    # --- Mechanism: hinge travel folds panels in the expected direction ---
    # Positive hinge_1 travel folds panel_1 further forward (-Y).
    rest_y1 = aabbs[1][0][1] if aabbs[1] else 0.0
    with ctx.pose({hinges[0]: 0.8}):
        fold_aabb = ctx.part_world_aabb(panels[1])
        ctx.check(
            "positive hinge_1 travel folds panel_1 further forward (-Y)",
            fold_aabb is not None and fold_aabb[0][1] < rest_y1 - 0.05,
            details=f"rest_y={rest_y1:.3f}, "
            f"fold_y={fold_aabb[0][1] if fold_aabb else None}",
        )

    # Positive hinge_2 travel folds panel_2 forward from its backward rest.
    rest_y2 = aabbs[2][0][1] if aabbs[2] else 0.0
    with ctx.pose({hinges[1]: 0.8}):
        fold_aabb2 = ctx.part_world_aabb(panels[2])
        ctx.check(
            "positive hinge_2 travel folds panel_2 forward from backward rest",
            fold_aabb2 is not None and fold_aabb2[0][1] < rest_y2 - 0.05,
            details=f"rest_y={rest_y2:.3f}, "
            f"fold_y={fold_aabb2[0][1] if fold_aabb2 else None}",
        )

    return ctx.report()


object_model = build_object_model()
