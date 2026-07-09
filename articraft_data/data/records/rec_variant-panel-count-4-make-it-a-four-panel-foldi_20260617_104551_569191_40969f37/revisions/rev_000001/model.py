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
# Four-panel Chinese-style folding screen (accordion room divider).
#
# World frame: Z up, decorated front faces -Y.
# panel_0 is the fixed root, centered at the origin.
# Panels 1..3 chain to the right: each panel_i hinges to panel_{i-1}
# along a vertical seam with alternating fold sense (accordion zigzag).
#
# Hinge convention: axis = (0, 0, -1) for all hinges. Positive q folds
# the child panel forward (toward parent's -Y). Odd hinges (0→1, 2→3)
# bake a forward rest angle; even hinge (1→2) bakes a backward rest
# angle, producing the characteristic accordion zigzag at rest.
# ---------------------------------------------------------------------------

N_PANELS = 4
PANEL_W = 0.60
PANEL_H = 1.70
PANEL_T = 0.03
RAIL_W = 0.05
BOTTOM_RAIL_H = 0.25
TOP_RAIL_H = 0.05
SEAM_GAP = 0.013
HINGE_Z_REF = 0.35  # lower hinge height (part-frame anchor for chain panels)
HINGE_HEIGHTS = (0.35, 1.35)
FOLD_ANGLE = math.radians(35.0)
FOLD_RANGE = math.radians(150.0)

OPEN_W = PANEL_W - 2.0 * RAIL_W  # 0.50 lattice opening width
OPEN_H = PANEL_H - BOTTOM_RAIL_H - TOP_RAIL_H  # 1.40 opening height
OPEN_ZC = BOTTOM_RAIL_H + OPEN_H / 2.0  # 0.95 opening center height

FRET_BAR = 0.015
FRET_DEPTH = 0.008

# Hinge seam x-positions in the parent panel's local frame.
ROOT_HINGE_X = PANEL_W / 2.0 + SEAM_GAP  # 0.313 (root panel is centered)
CHAIN_HINGE_X = SEAM_GAP + PANEL_W + SEAM_GAP  # 0.626 (chain panels extend from x=SEAM_GAP)


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _fret_lattice() -> cq.Workplane:
    """One panel's gold fretwork: interlocking rectangles and squares in XY,
    extruded by FRET_DEPTH. Same motif for every panel."""
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
    dz shifts nominal floor-based heights into the part frame."""
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


def _add_barrel_hardware(part, hinge_x: float, dz: float, extend_dir: float,
                         mats: dict) -> None:
    """Hinge barrel, web, and leaf on a child panel at the hinge axis.
    extend_dir = +1 when the panel extends in +x from the hinge."""
    brass = mats["brass"]
    for idx, zh in enumerate(HINGE_HEIGHTS):
        zl = zh - dz
        part.visual(
            Cylinder(radius=0.012, length=0.10),
            origin=Origin(xyz=(hinge_x, 0.0, zl)),
            material=brass,
            name=f"barrel_{idx}",
        )
        part.visual(
            Box((0.024, 0.022, 0.05)),
            origin=Origin(xyz=(hinge_x + extend_dir * 0.012, 0.0, zl)),
            material=brass,
            name=f"web_{idx}",
        )
        part.visual(
            Box((0.05, 0.005, 0.04)),
            origin=Origin(xyz=(hinge_x + extend_dir * 0.025, -0.017, zl)),
            material=brass,
            name=f"barrel_leaf_{idx}",
        )


def _add_knuckle_hardware(part, hinge_x: float, dz: float, mats: dict) -> None:
    """Hinge knuckles and leaf on a parent panel's right side of the seam."""
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


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_panel_folding_screen")

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
            # Root panel: centered at origin, standing on the floor.
            _add_panel(panel, -PANEL_W / 2.0, PANEL_W / 2.0, 0.0, mats, fret_mesh)
            # Knuckles on the right side for hinge to panel_1.
            _add_knuckle_hardware(panel, ROOT_HINGE_X, 0.0, mats)
        else:
            # Chain panel: extends from hinge axis (local x=0) in +x.
            # Part frame z=0 sits at the lower hinge height (world z=0.35).
            _add_panel(panel, SEAM_GAP, SEAM_GAP + PANEL_W, HINGE_Z_REF, mats, fret_mesh)
            # Barrel on the left (hinge axis = part frame origin).
            _add_barrel_hardware(panel, 0.0, HINGE_Z_REF, 1.0, mats)
            # Knuckles on the right for hinge to panel_{i+1} (if not last).
            if i < N_PANELS - 1:
                _add_knuckle_hardware(panel, CHAIN_HINGE_X, HINGE_Z_REF, mats)

        panels.append(panel)

    # --- Chained vertical-axis hinges with alternating fold sense. ---
    for i in range(1, N_PANELS):
        parent = panels[i - 1]
        child = panels[i]

        # Hinge seam x in parent's local frame.
        hinge_x = ROOT_HINGE_X if i == 1 else CHAIN_HINGE_X
        # Hinge z in parent's local frame (root has dz=0, chain has dz=HINGE_Z_REF).
        hinge_z = HINGE_Z_REF if i == 1 else 0.0

        # Alternating fold: odd hinges fold forward, even fold backward.
        forward = (i % 2 == 1)
        fold_sign = -1.0 if forward else 1.0

        if forward:
            lower = -(FOLD_RANGE + FOLD_ANGLE)
            upper = FOLD_RANGE - FOLD_ANGLE
        else:
            lower = -(FOLD_RANGE - FOLD_ANGLE)
            upper = FOLD_RANGE + FOLD_ANGLE

        model.articulation(
            f"hinge_{i - 1}_{i}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(
                xyz=(hinge_x, 0.0, hinge_z),
                rpy=(0.0, 0.0, fold_sign * FOLD_ANGLE),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=30.0, velocity=2.0, lower=lower, upper=upper,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    hinges = [
        object_model.get_articulation(f"hinge_{i - 1}_{i}")
        for i in range(1, N_PANELS)
    ]

    # --- Hero structure: four full-height panels standing on the floor. ---
    for panel in panels:
        aabb = ctx.part_world_aabb(panel)
        ctx.check(
            f"{panel.name} stands on the floor at full height",
            aabb is not None
            and -0.002 <= aabb[0][2] <= 0.01
            and 1.68 <= aabb[1][2] <= 1.72,
            details=f"aabb={aabb}",
        )

    # --- Accordion zigzag at rest: panels 0 and 2 are thin slabs (nearly
    # coplanar, facing -Y); panels 1 and 3 are angled forward with a large
    # Y footprint from the ~35 deg fold. ---
    aabbs = [ctx.part_world_aabb(p) for p in panels]

    def y_extent(aabb):
        return aabb[1][1] - aabb[0][1] if aabb else 0.0

    ctx.check(
        "panel_0 is a thin upright slab near Y=0",
        aabbs[0] is not None
        and aabbs[0][0][1] > -0.05
        and aabbs[0][1][1] < 0.05,
        details=f"panel_0={aabbs[0]}",
    )
    ctx.check(
        "panel_1 angles forward (large Y footprint from fold)",
        aabbs[1] is not None and y_extent(aabbs[1]) > 0.20,
        details=f"panel_1={aabbs[1]}, y_ext={y_extent(aabbs[1]):.3f}",
    )
    ctx.check(
        "panel_2 zigzags back to a thin slab (nearly coplanar with panel_0)",
        aabbs[2] is not None and y_extent(aabbs[2]) < 0.10,
        details=f"panel_2={aabbs[2]}, y_ext={y_extent(aabbs[2]):.3f}",
    )
    ctx.check(
        "panel_3 angles forward again (large Y footprint)",
        aabbs[3] is not None and y_extent(aabbs[3]) > 0.20,
        details=f"panel_3={aabbs[3]}, y_ext={y_extent(aabbs[3]):.3f}",
    )
    # Chain extends to the right: each panel is further right than the previous.
    for i in range(1, N_PANELS):
        ctx.check(
            f"panel_{i} is to the right of panel_{i - 1}",
            aabbs[i] is not None
            and aabbs[i - 1] is not None
            and aabbs[i][0][0] > aabbs[i - 1][0][0] + 0.10,
            details=f"panel_{i - 1}={aabbs[i - 1]}, panel_{i}={aabbs[i]}",
        )

    # Four-panel accordion spans wider than 1.5 m.
    ctx.check(
        "four-panel accordion spans at least 1.5 m",
        aabbs[0] is not None
        and aabbs[3] is not None
        and (aabbs[3][1][0] - aabbs[0][0][0]) >= 1.50,
        details=f"span={aabbs[3][1][0] - aabbs[0][0][0] if aabbs[0] and aabbs[3] else None}",
    )

    # --- Barrel/knuckle overlap allowances (captured hinge pins). ---
    for i in range(1, N_PANELS):
        parent = panels[i - 1]
        child = panels[i]
        for idx in range(len(HINGE_HEIGHTS)):
            for k in range(2):
                ctx.allow_overlap(
                    child,
                    parent,
                    elem_a=f"barrel_{idx}",
                    elem_b=f"knuckle_{idx}_{k}",
                    reason=(
                        f"Parent knuckle captures child barrel at hinge {i - 1}→{i} "
                        "around the shared vertical pin axis."
                    ),
                )
            ctx.expect_contact(
                child,
                parent,
                elem_a=f"barrel_{idx}",
                elem_b=f"knuckle_{idx}_0",
                name=f"panel_{i} barrel {idx} seated in panel_{i - 1} knuckle",
            )

    # --- Gold fretwork present on every panel. ---
    for panel in panels:
        fret = ctx.part_element_world_aabb(panel, elem="fretwork")
        field = ctx.part_element_world_aabb(panel, elem="lattice_field")
        ctx.check(
            f"{panel.name} fretwork stays framed inside the black field",
            field is not None
            and fret is not None
            and fret[0][0] >= field[0][0] - 0.002
            and fret[1][0] <= field[1][0] + 0.002
            and fret[0][2] >= field[0][2] - 0.002
            and fret[1][2] <= field[1][2] + 0.002,
            details=f"field={field}, fret={fret}",
        )

    # --- Bottom rail is about 0.25 m tall on the root panel. ---
    rail = ctx.part_element_world_aabb(panels[0], elem="bottom_rail")
    ctx.check(
        "solid bottom rail is about 0.25 m tall from the floor",
        rail is not None
        and rail[0][2] <= 0.002
        and 0.24 <= (rail[1][2] - rail[0][2]) <= 0.26,
        details=f"bottom_rail={rail}",
    )

    # --- Mechanism: positive hinge travel folds all panels further forward. ---
    # Pose all hinges to +1.0 rad (about 57 deg additional forward fold).
    pose_fwd = {h: 1.0 for h in hinges}
    with ctx.pose(pose_fwd):
        aabbs_fwd = [ctx.part_world_aabb(p) for p in panels]
        # Panel 1 should fold much further forward than at rest.
        ctx.check(
            "positive hinge travel folds panel_1 further forward",
            aabbs_fwd[1] is not None
            and aabbs[1] is not None
            and aabbs_fwd[1][0][1] < aabbs[1][0][1] - 0.10,
            details=f"rest={aabbs[1]}, posed={aabbs_fwd[1]}",
        )
        # All panels should remain upright (z extent near full height).
        for idx, panel in enumerate(panels):
            ctx.check(
                f"{panel.name} stays upright during forward fold",
                aabbs_fwd[idx] is not None
                and aabbs_fwd[idx][1][2] >= 1.60,
                details=f"posed={aabbs_fwd[idx]}",
            )

    # --- Negative hinge travel folds panels backward. ---
    pose_back = {h: -1.5 for h in hinges}
    with ctx.pose(pose_back):
        aabbs_back = [ctx.part_world_aabb(p) for p in panels]
        # Panel 1 should swing behind the screen (toward +Y).
        ctx.check(
            "negative hinge travel folds panel_1 behind the screen",
            aabbs_back[1] is not None
            and aabbs_back[1][1][1] > 0.30,
            details=f"posed={aabbs_back[1]}",
        )

    return ctx.report()


object_model = build_object_model()
