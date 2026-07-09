from __future__ import annotations

"""Roman blind with stacked fabric folds.

Fork from horizontal wooden venetian blind parent.
Change: replaces tilting wooden slats with stacked roman fabric folds — six
horizontal fold panels with curved (sinusoidal-bulge) cross-sections that read
as soft draped fabric ribs.

Keeps: headrail box, bottom rail lift direction, cord lock, pull cords, tassels.
Removes: tilt mechanism, tilt wand, ladder tapes.

Articulations:
- lift (PRISMATIC driver on the bottom rail) + 6 per-fold mimic lift joints:
  pulling the lift cord raises the bottom rail, and the fold panels themselves
  rise and gather into a tight stack under the headrail. Each fold rides its
  own prismatic carrier that mimics `lift`; the multiplier grows toward the
  bottom of the blind, so the lowest folds travel farthest and the gather is
  led from the bottom upward, collapsing the open ~180 mm fold pitch down to a
  ~30 mm stacked pitch directly beneath the headrail.

Support strategy: thin vertical lift-cord strips owned by the headrail run from
inside the headrail down past the fold panels. Every fold panel penetrates the
cord strips by ~0.5 mm on Y for physical-contact support.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BLIND_WIDTH = 0.80
FOLD_WIDTH = 0.78
FOLD_COUNT = 6
FOLD_HEIGHT = 0.155       # visible fabric height per fold panel
FOLD_DEPTH = 0.032        # forward bulge at mid-height (the fold rib)
FOLD_THICK = 0.004        # fabric panel back-face offset
FOLD_PITCH = 0.180        # fold-to-fold spacing when lowered
TOP_FOLD_Z = 1.080        # top fold center when lowered

HEADRAIL_SIZE = (BLIND_WIDTH, 0.062, 0.060)
HEADRAIL_Z = 1.27         # center; spans 1.24 .. 1.30

BOTTOM_RAIL_SIZE = (FOLD_WIDTH, 0.045, 0.022)
BOTTOM_RAIL_Z = 0.065     # center when lowered

# Lift cord strip stations (on the back of the blind, like roman-blind ring cords)
CORD_STATIONS = (-0.20, 0.20)
CORD_STRIP_W = 0.006
CORD_STRIP_T = 0.002
CORD_STRIP_Y = -0.0045    # 0.5 mm embed into fold back face at y = -FOLD_THICK

# ---------------------------------------------------------------- gather (lift)
STACK_PITCH = 0.030       # compressed fold-to-fold when fully raised
STACK_TOP_Z = 1.150       # top fold center when fully raised
RAIL_GATHER_GAP = 0.006   # clearance between gathered bottom fold and rail top


def fold_down_z(index: int) -> float:
    """Headrail-frame Z of a fold center when lowered. 1-based from top."""
    return TOP_FOLD_Z - (index - 1) * FOLD_PITCH


def fold_up_z(index: int) -> float:
    """Headrail-frame Z of a fold center when fully gathered under the headrail."""
    return STACK_TOP_Z - (index - 1) * STACK_PITCH


def fold_displacement(index: int) -> float:
    """How far a fold rises between lowered and fully gathered (always > 0)."""
    return fold_up_z(index) - fold_down_z(index)


# Bottom rail sits just under the lowest gathered fold at full lift.
RAIL_UP_Z = fold_up_z(FOLD_COUNT) - (
    0.5 * FOLD_HEIGHT + RAIL_GATHER_GAP + 0.5 * BOTTOM_RAIL_SIZE[2]
)
RAIL_TRAVEL = RAIL_UP_Z - BOTTOM_RAIL_Z

# ---------------------------------------------------------------- materials
HEADRAIL_WOOD = Material("headrail_wood", rgba=(0.55, 0.42, 0.30, 1.0))
BOTTOM_RAIL_WOOD = Material("bottom_rail_wood", rgba=(0.50, 0.38, 0.27, 1.0))
FABRIC_CREAM = Material("fabric_cream", rgba=(0.93, 0.89, 0.82, 1.0))
CORD_IVORY = Material("cord_ivory", rgba=(0.88, 0.82, 0.72, 1.0))
CORD_LOCK_WOOD = Material("cord_lock_wood", rgba=(0.45, 0.35, 0.25, 1.0))
TASSEL_WOOD = Material("tassel_wood", rgba=(0.52, 0.40, 0.28, 1.0))


# ---------------------------------------------------------------- geometry helper
def _roman_fold_panel(width: float, height: float, depth: float,
                      thickness: float = 0.004, n_curve: int = 14) -> MeshGeometry:
    """Build a roman fold panel mesh centered at origin.

    Width along X, height along Z, depth (forward bulge) along +Y.
    The front face has a smooth sinusoidal bulge peaking at mid-height — this
    bulge IS the visible horizontal fold rib. The back face is flat.
    Two end caps close the mesh into a solid.
    """
    mesh = MeshGeometry()
    hw, hh = width / 2.0, height / 2.0

    # Profile in YZ: front curve (top → bottom) then back (bottom → top)
    profile: list[tuple[float, float]] = []
    for i in range(n_curve + 1):
        t = i / n_curve
        z = hh * (1.0 - 2.0 * t)
        # Smooth sinusoidal bulge: zero at top/bottom edges, max at center
        y = depth * math.sin(t * math.pi)
        profile.append((y, z))
    profile.append((-thickness, -hh))   # bottom-back corner
    profile.append((-thickness, hh))     # top-back corner (closes loop)

    n = len(profile)

    # Left ring (x = -hw) and right ring (x = +hw)
    for y, z in profile:
        mesh.add_vertex(-hw, y, z)
    for y, z in profile:
        mesh.add_vertex(hw, y, z)

    # Side faces connecting left ring to right ring
    for i in range(n):
        j = (i + 1) % n
        mesh.add_face(i, j, n + j)
        mesh.add_face(i, n + j, n + i)

    # Left end cap (fan from center)
    lc = mesh.add_vertex(-hw, -thickness / 2.0, 0.0)
    for i in range(n):
        j = (i + 1) % n
        mesh.add_face(lc, j, i)

    # Right end cap
    rc = mesh.add_vertex(hw, -thickness / 2.0, 0.0)
    for i in range(n):
        j = (i + 1) % n
        mesh.add_face(rc, n + i, n + j)

    return mesh


# ============================================================= build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roman_blind")

    # --------------------------------------------------------- headrail (root)
    headrail = model.part("headrail")
    headrail.visual(
        Box(HEADRAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, HEADRAIL_Z)),
        material=HEADRAIL_WOOD,
        name="headrail_box",
    )

    # Lift cord strips: thin vertical strips on the back of the blind, running
    # from inside the headrail down past all folds. These represent the roman
    # blind's ring-cord channels and provide visual support for the fold panels.
    cord_len = HEADRAIL_Z - BOTTOM_RAIL_Z + 0.5 * BOTTOM_RAIL_SIZE[2]
    cord_mid_z = 0.5 * (HEADRAIL_Z + BOTTOM_RAIL_Z - 0.5 * BOTTOM_RAIL_SIZE[2])
    for station_x, idx in zip(CORD_STATIONS, range(2)):
        headrail.visual(
            Box((CORD_STRIP_W, CORD_STRIP_T, cord_len)),
            origin=Origin(xyz=(station_x, CORD_STRIP_Y, cord_mid_z)),
            material=CORD_IVORY,
            name=f"lift_cord_strip_{idx}",
        )

    # Cord lock on front-right of headrail
    headrail.visual(
        Box((0.07, 0.014, 0.030)),
        origin=Origin(xyz=(0.32, 0.036, 1.252)),
        material=CORD_LOCK_WOOD,
        name="cord_lock",
    )

    # Pull cords with tassels hanging at the front
    for cord_x, idx in ((0.305, 0), (0.335, 1)):
        cord_top = 1.260
        cord_bottom = 0.500
        headrail.visual(
            Cylinder(radius=0.0022, length=cord_top - cord_bottom),
            origin=Origin(xyz=(cord_x, 0.038, 0.5 * (cord_top + cord_bottom))),
            material=CORD_IVORY,
            name=f"pull_cord_{idx}",
        )
        headrail.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(cord_x, 0.038, 0.475)),
            material=TASSEL_WOOD,
            name=f"cord_tassel_{idx}",
        )

    # --------------------------------------------------------- bottom rail
    bottom_rail = model.part("bottom_rail")
    bottom_rail.visual(
        Box(BOTTOM_RAIL_SIZE),
        origin=Origin(),
        material=BOTTOM_RAIL_WOOD,
        name="bottom_rail_bar",
    )
    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=headrail,
        child=bottom_rail,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_RAIL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.5, lower=0.0, upper=RAIL_TRAVEL
        ),
    )

    # --------------------------------------------------------- fold panels
    # Each fold = a single part with a curved mesh visual, connected to the
    # headrail via a prismatic lift mimic. The bottom rail is the driver.
    for i in range(1, FOLD_COUNT + 1):
        fold = model.part(f"fold_{i}")

        fold_mesh = _roman_fold_panel(
            FOLD_WIDTH, FOLD_HEIGHT, FOLD_DEPTH, FOLD_THICK
        )
        fold.visual(
            mesh_from_geometry(fold_mesh, f"fold_panel_{i}"),
            origin=Origin(),
            material=FABRIC_CREAM,
            name=f"fold_panel_{i}",
        )

        disp = fold_displacement(i)
        lift_mult = disp / RAIL_TRAVEL
        model.articulation(
            f"fold_lift_{i}",
            ArticulationType.PRISMATIC,
            parent=headrail,
            child=fold,
            origin=Origin(xyz=(0.0, 0.0, fold_down_z(i))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=0.5, lower=0.0, upper=disp
            ),
            mimic=Mimic(joint="lift", multiplier=lift_mult, offset=0.0),
        )

    return model


# ============================================================= tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headrail = object_model.get_part("headrail")
    bottom_rail = object_model.get_part("bottom_rail")
    folds = [object_model.get_part(f"fold_{i}") for i in range(1, FOLD_COUNT + 1)]
    lift = object_model.get_articulation("lift")

    # 1. Fold count.
    fold_parts = [p for p in object_model.parts if p.name.startswith("fold_")]
    ctx.check(
        "fold_count_is_6",
        len(fold_parts) == FOLD_COUNT,
        f"expected {FOLD_COUNT} folds, found {len(fold_parts)}",
    )

    # 2. No tilt articulations (roman blinds don't tilt).
    tilt_joints = [a for a in object_model.articulations if "tilt" in a.name]
    ctx.check(
        "no_tilt_articulations",
        len(tilt_joints) == 0,
        f"found {len(tilt_joints)} tilt joints; roman blind should have none",
    )

    # 3. Lift driver: prismatic, vertical Z, positive stroke.
    ctx.check(
        "lift_driver_is_prismatic_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.articulation_type == ArticulationType.PRISMATIC
        and lift.mimic is None,
        f"lift axis={lift.axis} type={lift.articulation_type}",
    )
    ctx.check(
        "lift_stroke_positive",
        lift.motion_limits is not None and lift.motion_limits.upper > 0.3,
        f"lift travel={RAIL_TRAVEL:.3f} m",
    )

    # 4. Per-fold lift mimic chain: all mimic `lift`, multipliers strictly
    #    increase from top fold to bottom fold (bottom leads the gather).
    lift_followers = [
        object_model.get_articulation(f"fold_lift_{i}")
        for i in range(1, FOLD_COUNT + 1)
    ]
    mults = [a.mimic.multiplier for a in lift_followers if a.mimic is not None]
    ok_chain = (
        len(mults) == FOLD_COUNT
        and all(
            a.mimic is not None and a.mimic.joint == "lift"
            for a in lift_followers
        )
        and all(0.0 < mm <= 1.0 for mm in mults)
        and all(mults[k] < mults[k + 1] for k in range(len(mults) - 1))
    )
    ctx.check(
        "fold_lift_mimic_bottom_leads",
        ok_chain,
        f"multipliers={[round(m, 3) for m in mults]}",
    )

    # 5. Fold panels have curved geometry (Y extent exceeds flat thickness).
    for i, fold in enumerate(folds):
        aabb = ctx.part_world_aabb(fold)
        assert aabb is not None
        y_extent = aabb[1][1] - aabb[0][1]
        ctx.check(
            f"fold_{i + 1}_has_curved_rib_profile",
            y_extent > FOLD_THICK + 0.015,
            f"fold Y extent={y_extent:.4f} m (should exceed {FOLD_THICK} + bulge)",
        )

    # 6. Two lift cord strip stations at distinct X positions.
    strip_xs: set[float] = set()
    for idx in range(2):
        strip = headrail.get_visual(f"lift_cord_strip_{idx}")
        ctx.check(
            f"lift_cord_strip_{idx}_exists",
            strip is not None,
            "missing lift cord strip",
        )
        aabb = ctx.part_element_world_aabb(headrail, elem=strip)
        assert aabb is not None
        strip_xs.add(round(0.5 * (aabb[0][0] + aabb[1][0]), 4))
    ctx.check(
        "lift_cord_strips_at_two_stations",
        len(strip_xs) == 2,
        f"cord strip stations at x={sorted(strip_xs)}",
    )

    # 7. Folds span both cord strip stations.
    fold_aabb = ctx.part_world_aabb(folds[0])
    assert fold_aabb is not None
    ctx.check(
        "folds_span_both_cord_stations",
        all(fold_aabb[0][0] < x < fold_aabb[1][0] for x in strip_xs),
        f"fold x-range=({fold_aabb[0][0]:.3f},{fold_aabb[1][0]:.3f}), "
        f"stations={sorted(strip_xs)}",
    )

    headrail_box = headrail.get_visual("headrail_box")
    headrail_box_aabb = ctx.part_element_world_aabb(headrail, elem=headrail_box)
    assert headrail_box_aabb is not None

    # 8. Lowered pose: bottom rail below lowest fold, headrail above top fold,
    #    cord strips contact the bottom rail.
    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            folds[-1],
            bottom_rail,
            axis="z",
            min_gap=0.01,
            name="bottom_rail_below_lowest_fold",
        )
        ctx.expect_gap(
            headrail,
            folds[0],
            axis="z",
            min_gap=0.01,
            positive_elem=headrail_box,
            name="headrail_above_top_fold",
        )
        strip_0 = headrail.get_visual("lift_cord_strip_0")
        ctx.expect_contact(
            headrail,
            bottom_rail,
            elem_a=strip_0,
            name="lift_cord_strip_reaches_bottom_rail",
        )

    # 9. Raised pose: folds gather under headrail, bottom-led.
    travel = float(lift.motion_limits.upper)
    z_lowered = [ctx.part_world_position(f)[2] for f in folds]
    with ctx.pose({lift: travel}):
        z_raised = [ctx.part_world_position(f)[2] for f in folds]
        rises = [zr - zl for zr, zl in zip(z_raised, z_lowered)]

        ctx.check(
            "all_folds_rise_on_lift",
            all(r > 1e-4 for r in rises),
            f"min rise={min(rises):.4f} m",
        )
        ctx.check(
            "lower_folds_lead_the_gather",
            all(rises[k] < rises[k + 1] for k in range(len(rises) - 1)),
            f"rises top->bottom={[round(r, 3) for r in rises]}",
        )

        stack_span = z_raised[0] - z_raised[-1]
        expected_span = (FOLD_COUNT - 1) * STACK_PITCH
        ctx.check(
            "gathered_stack_is_tight",
            abs(stack_span - expected_span) < 0.02,
            f"stack span={stack_span:.3f} m, expected ~{expected_span:.3f} m",
        )

        top_fold_aabb = ctx.part_world_aabb(folds[0])
        assert top_fold_aabb is not None
        ctx.check(
            "gathered_stack_under_headrail",
            top_fold_aabb[1][2] < headrail_box_aabb[0][2],
            f"top fold top={top_fold_aabb[1][2]:.3f}, "
            f"headrail bottom={headrail_box_aabb[0][2]:.3f}",
        )

        ctx.expect_gap(
            folds[-1],
            bottom_rail,
            axis="z",
            min_gap=0.002,
            name="bottom_rail_under_gathered_stack",
        )

    # 10. Pull cords + tassels hang at front, below headrail.
    for idx in (0, 1):
        tassel = headrail.get_visual(f"cord_tassel_{idx}")
        aabb = ctx.part_element_world_aabb(headrail, elem=tassel)
        assert aabb is not None
        ctx.check(
            f"tassel_{idx}_hangs_below_headrail_front",
            aabb[1][2] < headrail_box_aabb[0][2] - 0.5 and aabb[0][1] > 0.0,
            f"tassel aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
