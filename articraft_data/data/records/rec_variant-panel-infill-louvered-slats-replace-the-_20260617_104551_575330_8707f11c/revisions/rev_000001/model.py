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
)

# ---------------------------------------------------------------------------
# Three-panel Chinese-style folding screen (room divider).
#
# World frame: Z up, the decorated front of the screen faces -Y.
# Center panel is the fixed root, spanning x in [-0.30, +0.30].
# Each wing panel hangs on a vertical revolute hinge at the seam
# (x = +/-0.313, leaving a 13 mm hinge gap), with the photo pose
# (~35 deg folded forward) baked into the articulation origin so the
# rest pose matches the reference image. Joint range covers roughly
# -150..+150 deg from coplanar so each wing folds nearly flat either way.
# ---------------------------------------------------------------------------

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

SLAT_FACE = 0.055   # slat board face height (visible width when tilted)
SLAT_THICK = 0.006  # slat board thickness
SLAT_TILT = math.radians(35)  # fixed tilt from horizontal (front edge up)
N_SLATS = 20        # number of louver slats per panel opening
HINGE_HEIGHTS = (0.35, 1.35)


def _louver_slat_geometry() -> Box:
    """One louver slat: a thin rectangular board spanning the panel opening
    width, with SLAT_FACE height and SLAT_THICK depth. The board is placed
    in its own centered frame; tilt is applied at placement via Origin.rpy.
    """
    return Box((OPEN_W + 0.01, SLAT_THICK, SLAT_FACE))


def _add_panel(part, x0: float, x1: float, dz: float, mats: dict) -> None:
    """Author one louvered screen panel spanning part-local x in [x0, x1].

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
    slat_mat = mats["slat_wood"]

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
    # Matte black inset field backing the louver slats (thin, centered in the
    # frame thickness, with a 5 mm embed into rails/stiles so the panel reads
    # as one assembly).
    part.visual(
        Box((OPEN_W + 0.01, 0.012, OPEN_H + 0.01)),
        origin=Origin(xyz=(cx, 0.0, z(OPEN_ZC))),
        material=black,
        name="louver_field",
    )
    # Fixed horizontal louver slats filling the opening, evenly spaced
    # vertically. Each slat is a thin board tilted ~35 deg (front edge up)
    # so the panel reads as a louvered room divider.
    slat_geom = _louver_slat_geometry()
    slat_spacing = OPEN_H / (N_SLATS + 1)
    for i in range(N_SLATS):
        sz = (OPEN_ZC - OPEN_H / 2.0) + slat_spacing * (i + 1)
        part.visual(
            slat_geom,
            origin=Origin(xyz=(cx, 0.0, z(sz)), rpy=(SLAT_TILT, 0.0, 0.0)),
            material=slat_mat,
            name=f"slat_{i}",
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_panel_folding_screen")

    mats = {
        "lacquer": model.material("red_brown_lacquer", rgba=(0.38, 0.10, 0.08, 1.0)),
        "worn_lacquer": model.material("worn_red_lacquer", rgba=(0.45, 0.15, 0.11, 1.0)),
        "black_field": model.material("matte_black_field", rgba=(0.05, 0.05, 0.06, 1.0)),
        "slat_wood": model.material("warm_bamboo", rgba=(0.55, 0.38, 0.18, 1.0)),
        "brass": model.material("aged_brass", rgba=(0.55, 0.42, 0.18, 1.0)),
    }

    # Fixed center panel (root), standing on the floor.
    center = model.part("center_panel")
    _add_panel(center, -PANEL_W / 2.0, PANEL_W / 2.0, 0.0, mats)
    # Center-side hinge leaf plates near each seam, on the front face, plus
    # fixed knuckles that reach across the seam gap and capture the hinge
    # barrels (piano-hinge construction; the wing barrel spins inside them).
    for side, s in enumerate((1.0, -1.0)):
        for idx, zh in enumerate(HINGE_HEIGHTS):
            center.visual(
                Box((0.036, 0.005, 0.07)),
                origin=Origin(xyz=(s * 0.280, -0.017, zh)),
                material=mats["brass"],
                name=f"hinge_leaf_{side}_{idx}",
            )
            for k, sz in enumerate((-1.0, 1.0)):
                center.visual(
                    Box((0.027, 0.022, 0.025)),
                    origin=Origin(xyz=(s * 0.3055, 0.0, zh + sz * 0.035)),
                    material=mats["brass"],
                    name=f"hinge_knuckle_{side}_{idx}_{k}",
                )

    # Wing panels: part frame sits on the hinge axis at the seam, with the
    # panel slab extending away from the axis (a 13 mm hinge gap keeps the
    # rotating edge clear of the center panel).
    wing_0 = model.part("wing_panel_0")
    _add_panel(wing_0, SEAM_GAP, SEAM_GAP + PANEL_W, HINGE_HEIGHTS[0], mats)
    _add_wing_hinge_hardware(wing_0, 1.0, HINGE_HEIGHTS[0], mats)

    wing_1 = model.part("wing_panel_1")
    _add_panel(wing_1, -SEAM_GAP - PANEL_W, -SEAM_GAP, HINGE_HEIGHTS[0], mats)
    _add_wing_hinge_hardware(wing_1, -1.0, HINGE_HEIGHTS[0], mats)

    # Vertical-axis hinges at the seams. The photo pose (35 deg forward of
    # coplanar) is baked into the origin rpy; axes are mirrored so positive q
    # folds each wing further forward (toward -Y). q spans the remaining
    # travel of the +/-150 deg-from-coplanar range on each side.
    lower = -(FOLD_RANGE + FOLD_ANGLE)  # back-fold limit (-150 deg coplanar)
    upper = FOLD_RANGE - FOLD_ANGLE  # forward-fold limit (+150 deg coplanar)
    model.articulation(
        "wing_hinge_0",
        ArticulationType.REVOLUTE,
        parent=center,
        child=wing_0,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_HEIGHTS[0]), rpy=(0.0, 0.0, -FOLD_ANGLE)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=lower, upper=upper),
    )
    model.articulation(
        "wing_hinge_1",
        ArticulationType.REVOLUTE,
        parent=center,
        child=wing_1,
        origin=Origin(xyz=(-HINGE_X, 0.0, HINGE_HEIGHTS[0]), rpy=(0.0, 0.0, FOLD_ANGLE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=lower, upper=upper),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center = object_model.get_part("center_panel")
    wing_0 = object_model.get_part("wing_panel_0")
    wing_1 = object_model.get_part("wing_panel_1")
    hinge_0 = object_model.get_articulation("wing_hinge_0")
    hinge_1 = object_model.get_articulation("wing_hinge_1")

    # --- Hero structure: three full-height panels standing on the floor. ---
    for part in (center, wing_0, wing_1):
        aabb = ctx.part_world_aabb(part)
        ctx.check(
            f"{part.name} stands on the floor at full height",
            aabb is not None
            and -0.002 <= aabb[0][2] <= 0.01
            and 1.68 <= aabb[1][2] <= 1.72,
            details=f"aabb={aabb}",
        )

    # Rest pose matches the photo: both wings angled forward (toward -Y),
    # center panel staying in its upright plane.
    aabb_c = ctx.part_world_aabb(center)
    aabb_w0 = ctx.part_world_aabb(wing_0)
    aabb_w1 = ctx.part_world_aabb(wing_1)
    ctx.check(
        "wings angle forward ~35 deg in the rest pose",
        aabb_w0 is not None
        and aabb_w1 is not None
        and aabb_w0[0][1] < -0.30
        and aabb_w1[0][1] < -0.30,
        details=f"wing_0={aabb_w0}, wing_1={aabb_w1}",
    )
    ctx.check(
        "center panel stays a thin upright slab",
        aabb_c is not None and aabb_c[0][1] > -0.05 and aabb_c[1][1] < 0.05,
        details=f"center={aabb_c}",
    )
    ctx.check(
        "partially folded screen spans roughly 1.6 m",
        aabb_w0 is not None
        and aabb_w1 is not None
        and 1.50 <= (aabb_w0[1][0] - aabb_w1[0][0]) <= 1.70,
        details=f"span={(aabb_w0[1][0] - aabb_w1[0][0]) if aabb_w0 and aabb_w1 else None}",
    )

    # Hinge barrels are intentionally captured inside the fixed center-panel
    # knuckles (piano-hinge pin/knuckle fit). The barrel sits on the hinge
    # axis, so the embed is pose-invariant while the wing rotates.
    for side, wing in ((0, wing_0), (1, wing_1)):
        for idx in range(len(HINGE_HEIGHTS)):
            for k in range(2):
                ctx.allow_overlap(
                    wing,
                    center,
                    elem_a=f"hinge_barrel_{idx}",
                    elem_b=f"hinge_knuckle_{side}_{idx}_{k}",
                    reason=(
                        "Fixed center-panel hinge knuckle captures the wing's "
                        "hinge barrel around the shared pin axis."
                    ),
                )
            ctx.expect_contact(
                wing,
                center,
                elem_a=f"hinge_barrel_{idx}",
                elem_b=f"hinge_knuckle_{side}_{idx}_0",
                name=f"{wing.name} barrel {idx} is seated in the center knuckle",
            )

    # Hinges sit on the panel seams (wing part frames lie on the hinge axes).
    p0 = ctx.part_world_position(wing_0)
    p1 = ctx.part_world_position(wing_1)
    ctx.check(
        "wing hinge axes sit at the panel seams",
        p0 is not None
        and p1 is not None
        and abs(p0[0] - 0.313) < 0.005
        and abs(p1[0] + 0.313) < 0.005
        and abs(p0[1]) < 0.005
        and abs(p1[1]) < 0.005,
        details=f"p0={p0}, p1={p1}",
    )
    for wing in (wing_0, wing_1):
        barrel = ctx.part_element_world_aabb(wing, elem="hinge_barrel_0")
        ctx.check(
            f"{wing.name} brass hinge barrel is centered on the seam axis",
            barrel is not None
            and abs((barrel[0][0] + barrel[1][0]) / 2.0) - 0.313 < 0.005
            and abs((barrel[0][1] + barrel[1][1]) / 2.0) < 0.005,
            details=f"barrel={barrel}",
        )

    # Louver slats: present on every panel, filling the frame opening with
    # evenly-spaced tilted boards.
    field_c = ctx.part_element_world_aabb(center, elem="louver_field")
    slat_first = ctx.part_element_world_aabb(center, elem="slat_0")
    slat_last = ctx.part_element_world_aabb(center, elem=f"slat_{N_SLATS - 1}")
    ctx.check(
        "center panel carries louver slats across the opening height",
        slat_first is not None
        and slat_last is not None
        and slat_first[0][2] <= 0.35
        and slat_last[1][2] >= 1.60,
        details=f"first={slat_first}, last={slat_last}",
    )
    ctx.check(
        "center louver slats stay within the black field backing (x)",
        field_c is not None
        and slat_first is not None
        and slat_first[0][0] >= field_c[0][0] - 0.005
        and slat_first[1][0] <= field_c[1][0] + 0.005,
        details=f"field={field_c}, slat_0={slat_first}",
    )
    # Slats are tilted, so their Y extent should be wider than a flat board.
    ctx.check(
        "center slats are tilted (Y extent wider than flat thickness)",
        slat_first is not None
        and (slat_first[1][1] - slat_first[0][1]) > 0.015,
        details=f"slat_0_y_extent={slat_first[1][1] - slat_first[0][1] if slat_first else None}",
    )
    for wing in (wing_0, wing_1):
        s0 = ctx.part_element_world_aabb(wing, elem="slat_0")
        s_last = ctx.part_element_world_aabb(wing, elem=f"slat_{N_SLATS - 1}")
        ctx.check(
            f"{wing.name} carries louver slats across the opening height",
            s0 is not None
            and s_last is not None
            and s0[0][2] <= 0.35
            and s_last[1][2] >= 1.60,
            details=f"first={s0}, last={s_last}",
        )

    # Solid bottom rail is clearly taller than the other rails (~0.25 m).
    rail_c = ctx.part_element_world_aabb(center, elem="bottom_rail")
    ctx.check(
        "solid bottom rail is about 0.25 m tall from the floor",
        rail_c is not None
        and rail_c[0][2] <= 0.002
        and 0.24 <= (rail_c[1][2] - rail_c[0][2]) <= 0.26,
        details=f"bottom_rail={rail_c}",
    )

    # --- Mechanism: each wing folds forward and backward about its seam. ---
    with ctx.pose({hinge_0: 1.31, hinge_1: 1.31}):  # ~110 deg forward of coplanar
        f0 = ctx.part_world_aabb(wing_0)
        f1 = ctx.part_world_aabb(wing_1)
        ctx.check(
            "positive hinge travel folds both wings forward across the front",
            f0 is not None
            and f1 is not None
            and f0[0][1] < -0.50
            and f1[0][1] < -0.50
            and f0[1][0] < 0.45
            and f1[0][0] > -0.45,
            details=f"wing_0={f0}, wing_1={f1}",
        )
    with ctx.pose({hinge_0: -2.18, hinge_1: -2.18}):  # ~90 deg behind coplanar
        b0 = ctx.part_world_aabb(wing_0)
        b1 = ctx.part_world_aabb(wing_1)
        ctx.check(
            "negative hinge travel folds both wings behind the screen",
            b0 is not None
            and b1 is not None
            and b0[1][1] > 0.50
            and b1[1][1] > 0.50,
            details=f"wing_0={b0}, wing_1={b1}",
        )

    return ctx.report()


object_model = build_object_model()
