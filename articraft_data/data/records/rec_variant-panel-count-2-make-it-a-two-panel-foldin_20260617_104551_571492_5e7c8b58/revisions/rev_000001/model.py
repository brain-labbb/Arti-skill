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
# Two-panel Chinese-style folding screen (room divider).
#
# World frame: Z up, the decorated front of the screen faces -Y.
# panel_0 is the fixed root, spanning x in [-0.30, +0.30].
# panel_1 is the wing, hinged at the right seam (x = +0.313, leaving a
# 13 mm hinge gap), with the photo pose (~35 deg folded forward) baked
# into the articulation origin so the rest pose matches a typical
# room-divider stance. Joint range covers roughly -150..+150 deg from
# coplanar so the wing folds nearly flat either way.
# ---------------------------------------------------------------------------

PANEL_COUNT = 2
PANEL_W = 0.60
PANEL_H = 1.70
PANEL_T = 0.03
RAIL_W = 0.05
BOTTOM_RAIL_H = 0.25
TOP_RAIL_H = 0.05
SEAM_GAP = 0.013
HINGE_X = PANEL_W / 2.0 + SEAM_GAP  # 0.313
FOLD_ANGLE = math.radians(35.0)  # photo rest pose, forward of coplanar
FOLD_RANGE = math.radians(150.0)  # max fold from coplanar, either way

OPEN_W = PANEL_W - 2.0 * RAIL_W  # 0.50 lattice opening width
OPEN_H = PANEL_H - BOTTOM_RAIL_H - TOP_RAIL_H  # 1.40 opening height
OPEN_ZC = BOTTOM_RAIL_H + OPEN_H / 2.0  # 0.95 opening center height

FRET_BAR = 0.015  # lattice bar width
FRET_DEPTH = 0.008  # lattice bar depth (3 mm embedded, 5 mm proud)
HINGE_HEIGHTS = (0.35, 1.35)


def _fret_lattice() -> cq.Workplane:
    """One panel's gold fretwork: a fused, connected lattice of interlocking
    rectangles and squares, drawn in the XY plane and extruded by FRET_DEPTH.

    Layout (u = panel width axis, v = panel height axis, centered on the
    black field): an outer border ring, three stacked motif clusters of
    concentric rectangles with bridge bars, interlocked corner squares,
    and connector bars tying every motif to the border ring.
    """
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

    solid = ring(0.0, 0.0, 0.49, 1.38)  # border ring
    for vc in (-0.43, 0.0, 0.43):
        solid = solid.union(ring(0.0, vc, 0.36, 0.32))  # cluster outer rect
        solid = solid.union(ring(0.0, vc, 0.20, 0.16))  # cluster inner rect
        for s in (1.0, -1.0):
            # inner->outer bridge bars (horizontal and vertical)
            solid = solid.union(bar(s * 0.14, vc, 0.09, t))
            solid = solid.union(bar(0.0, vc + s * 0.12, t, 0.09))
            # cluster->border connector bars
            solid = solid.union(bar(s * 0.205, vc, 0.06, t))
            for s2 in (1.0, -1.0):
                # interlocked corner squares on the cluster corners
                solid = solid.union(ring(s * 0.18, vc + s2 * 0.16, 0.09, 0.09))
    # vertical connectors between clusters and to the border top/bottom
    for vc in (-0.215, 0.215):
        solid = solid.union(bar(0.0, vc, t, 0.13))
    for vc in (-0.635, 0.635):
        solid = solid.union(bar(0.0, vc, t, 0.10))
    return solid


def _add_panel(part, x0: float, x1: float, dz: float, mats: dict, fret_mesh) -> None:
    """Author one lacquered screen panel spanning part-local x in [x0, x1].

    dz shifts nominal floor-based heights into the part frame (wing part
    frames sit on the lower hinge barrel at nominal z = 0.35).
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
    # Matte black inset field (thin, centered in the frame thickness, with a
    # 5 mm embed into rails/stiles so the panel reads as one assembly).
    part.visual(
        Box((OPEN_W + 0.01, 0.012, OPEN_H + 0.01)),
        origin=Origin(xyz=(cx, 0.0, z(OPEN_ZC))),
        material=black,
        name="lattice_field",
    )
    # Gold fretwork, raised proud of the front face of the black field.
    # Mesh local +Z (extrude direction) maps to world -Y via rpy x=+90 deg.
    part.visual(
        fret_mesh,
        origin=Origin(xyz=(cx, -0.003, z(OPEN_ZC)), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="fretwork",
    )


def _add_wing_hinge_hardware(wing, s: float, dz: float, mats: dict) -> None:
    """Brass barrel + leaf hardware on a wing part. The hinge axis is the
    wing part's local Z axis; s = +1 when the wing panel extends along +x.
    """
    brass = mats["brass"]
    for idx, zh in enumerate(HINGE_HEIGHTS):
        zl = zh - dz
        wing.visual(
            Cylinder(radius=0.012, length=0.10),
            origin=Origin(xyz=(0.0, 0.0, zl)),
            material=brass,
            name=f"hinge_barrel_{idx}",
        )
        # Hidden web tying the barrel to the wing stile across the seam gap.
        wing.visual(
            Box((0.024, 0.022, 0.05)),
            origin=Origin(xyz=(s * 0.012, 0.0, zl)),
            material=brass,
            name=f"hinge_web_{idx}",
        )
        # Visible leaf plate on the wing front face.
        wing.visual(
            Box((0.05, 0.005, 0.04)),
            origin=Origin(xyz=(s * 0.025, -0.017, zl)),
            material=brass,
            name=f"hinge_leaf_{idx}",
        )


def _add_fixed_hinge_hardware(fixed_panel, s: float, mats: dict) -> None:
    """Hinge knuckles and leaf plates on the fixed panel's seam edge.
    s = +1 for the right seam, -1 for the left seam.
    """
    brass = mats["brass"]
    for idx, zh in enumerate(HINGE_HEIGHTS):
        fixed_panel.visual(
            Box((0.036, 0.005, 0.07)),
            origin=Origin(xyz=(s * 0.280, -0.017, zh)),
            material=brass,
            name=f"hinge_leaf_{idx}",
        )
        for k, sz in enumerate((-1.0, 1.0)):
            fixed_panel.visual(
                Box((0.027, 0.022, 0.025)),
                origin=Origin(xyz=(s * 0.3055, 0.0, zh + sz * 0.035)),
                material=brass,
                name=f"hinge_knuckle_{idx}_{k}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_folding_screen")

    mats = {
        "lacquer": model.material("red_brown_lacquer", rgba=(0.38, 0.10, 0.08, 1.0)),
        "worn_lacquer": model.material("worn_red_lacquer", rgba=(0.45, 0.15, 0.11, 1.0)),
        "black_field": model.material("matte_black_field", rgba=(0.05, 0.05, 0.06, 1.0)),
        "gold": model.material("antique_gold", rgba=(0.78, 0.62, 0.22, 1.0)),
        "brass": model.material("aged_brass", rgba=(0.55, 0.42, 0.18, 1.0)),
    }

    fret_mesh = mesh_from_cadquery(_fret_lattice(), "fret_lattice")

    # Motion limits shared by every wing hinge (uniform hinge policy).
    lower = -(FOLD_RANGE + FOLD_ANGLE)  # back-fold limit
    upper = FOLD_RANGE - FOLD_ANGLE  # forward-fold limit

    panels = []
    for i in range(PANEL_COUNT):
        p = model.part(f"panel_{i}")
        panels.append(p)

        if i == 0:
            # Fixed root panel, centered at origin, standing on the floor.
            _add_panel(p, -PANEL_W / 2.0, PANEL_W / 2.0, 0.0, mats, fret_mesh)
        else:
            # Wing panel: part frame sits on the hinge axis at the right seam,
            # panel slab extends along +x from the hinge.
            _add_panel(p, SEAM_GAP, SEAM_GAP + PANEL_W, HINGE_HEIGHTS[0], mats, fret_mesh)
            _add_wing_hinge_hardware(p, 1.0, HINGE_HEIGHTS[0], mats)

            # Fixed panel gets hinge knuckles on its right seam edge.
            _add_fixed_hinge_hardware(panels[0], 1.0, mats)

            # Vertical-axis hinge at the right seam. The photo pose (35 deg
            # forward of coplanar) is baked into the origin rpy; axis chosen
            # so positive q folds the wing forward (toward -Y).
            model.articulation(
                f"hinge_{i}",
                ArticulationType.REVOLUTE,
                parent=panels[0],
                child=p,
                origin=Origin(
                    xyz=(HINGE_X, 0.0, HINGE_HEIGHTS[0]),
                    rpy=(0.0, 0.0, -FOLD_ANGLE),
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=30.0, velocity=2.0, lower=lower, upper=upper
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panels = [object_model.get_part(f"panel_{i}") for i in range(PANEL_COUNT)]
    fixed = panels[0]
    hinges = [
        object_model.get_articulation(f"hinge_{i}")
        for i in range(1, PANEL_COUNT)
    ]

    # --- Hero structure: two full-height panels standing on the floor. ---
    for p in panels:
        aabb = ctx.part_world_aabb(p)
        ctx.check(
            f"{p.name} stands on the floor at full height",
            aabb is not None
            and -0.002 <= aabb[0][2] <= 0.01
            and 1.68 <= aabb[1][2] <= 1.72,
            details=f"aabb={aabb}",
        )

    # Rest pose: wing angled forward (toward -Y), fixed panel staying upright.
    aabb_fixed = ctx.part_world_aabb(fixed)
    aabb_wing = ctx.part_world_aabb(panels[1])
    ctx.check(
        "wing panel angles forward ~35 deg in the rest pose",
        aabb_wing is not None and aabb_wing[0][1] < -0.30,
        details=f"wing={aabb_wing}",
    )
    ctx.check(
        "fixed panel stays a thin upright slab",
        aabb_fixed is not None
        and aabb_fixed[0][1] > -0.05
        and aabb_fixed[1][1] < 0.05,
        details=f"fixed={aabb_fixed}",
    )
    ctx.check(
        "partially folded two-panel screen spans roughly 1.0 to 1.2 m",
        aabb_fixed is not None
        and aabb_wing is not None
        and 0.95 <= (aabb_wing[1][0] - aabb_fixed[0][0]) <= 1.25,
        details=f"span={(aabb_wing[1][0] - aabb_fixed[0][0]) if aabb_fixed and aabb_wing else None}",
    )

    # Hinge barrels are intentionally captured inside the fixed-panel knuckles
    # (piano-hinge pin/knuckle fit). The barrel sits on the hinge axis, so the
    # embed is pose-invariant while the wing rotates.
    wing = panels[1]
    for idx in range(len(HINGE_HEIGHTS)):
        for k in range(2):
            ctx.allow_overlap(
                wing,
                fixed,
                elem_a=f"hinge_barrel_{idx}",
                elem_b=f"hinge_knuckle_{idx}_{k}",
                reason=(
                    "Fixed-panel hinge knuckle captures the wing's hinge barrel "
                    "around the shared pin axis."
                ),
            )
        ctx.expect_contact(
            wing,
            fixed,
            elem_a=f"hinge_barrel_{idx}",
            elem_b=f"hinge_knuckle_{idx}_0",
            name=f"wing barrel {idx} is seated in the fixed knuckle",
        )

    # Hinge sits on the panel seam (wing part frame lies on the hinge axis).
    p_wing = ctx.part_world_position(wing)
    ctx.check(
        "wing hinge axis sits at the right panel seam",
        p_wing is not None
        and abs(p_wing[0] - HINGE_X) < 0.005
        and abs(p_wing[1]) < 0.005,
        details=f"wing_pos={p_wing}",
    )
    barrel = ctx.part_element_world_aabb(wing, elem="hinge_barrel_0")
    ctx.check(
        "brass hinge barrel is centered on the seam axis",
        barrel is not None
        and abs((barrel[0][0] + barrel[1][0]) / 2.0 - HINGE_X) < 0.005
        and abs((barrel[0][1] + barrel[1][1]) / 2.0) < 0.005,
        details=f"barrel={barrel}",
    )

    # Gold fretwork: present on every panel, framed inside the black field.
    field_fixed = ctx.part_element_world_aabb(fixed, elem="lattice_field")
    fret_fixed = ctx.part_element_world_aabb(fixed, elem="fretwork")
    ctx.check(
        "fixed panel fretwork stays framed inside the black field (x/z)",
        field_fixed is not None
        and fret_fixed is not None
        and fret_fixed[0][0] >= field_fixed[0][0] - 0.001
        and fret_fixed[1][0] <= field_fixed[1][0] + 0.001
        and fret_fixed[0][2] >= field_fixed[0][2] - 0.001
        and fret_fixed[1][2] <= field_fixed[1][2] + 0.001,
        details=f"field={field_fixed}, fret={fret_fixed}",
    )
    ctx.check(
        "fixed panel fretwork bars are raised proud of the black field",
        field_fixed is not None
        and fret_fixed is not None
        and fret_fixed[0][1] < field_fixed[0][1] - 0.003,
        details=f"field_front={field_fixed[0][1] if field_fixed else None}, "
        f"fret_front={fret_fixed[0][1] if fret_fixed else None}",
    )
    fret_wing = ctx.part_element_world_aabb(wing, elem="fretwork")
    ctx.check(
        "wing panel carries its gold lattice across the opening height",
        fret_wing is not None
        and 0.20 <= fret_wing[0][2] <= 0.32
        and 1.58 <= fret_wing[1][2] <= 1.70,
        details=f"fret={fret_wing}",
    )

    # Solid bottom rail is clearly taller than the other rails (~0.25 m).
    rail = ctx.part_element_world_aabb(fixed, elem="bottom_rail")
    ctx.check(
        "solid bottom rail is about 0.25 m tall from the floor",
        rail is not None
        and rail[0][2] <= 0.002
        and 0.24 <= (rail[1][2] - rail[0][2]) <= 0.26,
        details=f"bottom_rail={rail}",
    )

    # --- Mechanism: the wing folds forward and backward about its seam. ---
    hinge_1 = hinges[0]
    with ctx.pose({hinge_1: 1.31}):  # ~110 deg forward of coplanar
        f = ctx.part_world_aabb(wing)
        ctx.check(
            "positive hinge travel folds the wing forward across the front",
            f is not None
            and f[0][1] < -0.50
            and f[1][0] < 0.50,
            details=f"wing={f}",
        )
    with ctx.pose({hinge_1: -2.18}):  # ~90 deg behind coplanar
        b = ctx.part_world_aabb(wing)
        ctx.check(
            "negative hinge travel folds the wing behind the screen",
            b is not None
            and b[1][1] > 0.50,
            details=f"wing={b}",
        )

    return ctx.report()


object_model = build_object_model()
