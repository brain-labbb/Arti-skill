from __future__ import annotations

from math import pi

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
# Shared cylinder helpers (preserved from parent baseline)
# ---------------------------------------------------------------------------

def _cyl_x(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


# ---------------------------------------------------------------------------
# Chain link helper — emits one chain link body for a given side and index
# ---------------------------------------------------------------------------

def _chain_link(part, side_idx: int, link_idx: int, x: float, z: float,
                link_height: float, material) -> None:
    """Emit a single chain link visual.

    Even-index links are flat-facing (wide in X, thin in Y) to read as the
    face of a chain link.  Odd-index links are edge-on (thin in X, wide in Y)
    to read as the perpendicular link in a real chain run.
    """
    link_width = 0.034
    link_thick = 0.007
    if link_idx % 2 == 0:
        # Flat link — face visible from the side
        part.visual(
            Box((link_width, link_thick, link_height)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=material,
            name=f"chain_link_{side_idx}_{link_idx}",
        )
    else:
        # Edge link — thin profile, wide in Y
        part.visual(
            Box((link_thick, link_width, link_height)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=material,
            name=f"chain_link_{side_idx}_{link_idx}",
        )


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="portable_chain_escape_ladder",
        meta={
            "run_notes": (
                "Portable roll-up chain-link fire escape ladder that hooks over "
                "a windowsill and drops down a building facade.  The two rigid "
                "steel rails of the parent fixed ladder are replaced by flexible "
                "chain side-members (loop-emitted chain_link_{side}_{i} "
                "primitives with alternating flat/edge orientations).  Rigid "
                "standoff rungs span between the chains.  The wall-plate / cage "
                "/ lower-slide hardware is replaced by a top sill-hook frame "
                "on a REVOLUTE pivot."
            ),
        },
    )

    # -- Materials --
    zinc_chain = model.material("zinc_plated_chain", rgba=(0.72, 0.76, 0.74, 1.0))
    rung_steel = model.material("rung_steel", rgba=(0.62, 0.66, 0.64, 1.0))
    hook_steel = model.material("hook_frame_steel", rgba=(0.50, 0.54, 0.52, 1.0))
    dark_pin = model.material("pivot_pin", rgba=(0.12, 0.13, 0.13, 1.0))

    # -- Ladder geometry parameters --
    ladder_height = 4.20          # deployed length
    chain_x = 0.150               # half-width between chain centre-lines
    num_links = 28                # links per side
    link_pitch = ladder_height / num_links  # ~0.15 m
    num_rungs = 13                # climbing rungs
    rung_spacing = ladder_height / (num_rungs + 1)  # ~0.30 m
    standoff_length = 0.080      # wall-standoff stub length

    # =====================================================================
    # Root part: upper_ladder  (chains + rungs + standoff stubs + head bar)
    # =====================================================================
    upper = model.part("upper_ladder")

    # Chain link height equals the pitch so consecutive links touch end-to-end,
    # forming a geometrically connected chain within the part.
    link_height = link_pitch

    # --- Chain side-members (loop-emitted) ---
    for side_idx, sx in enumerate((-chain_x, chain_x)):
        for link_idx in range(num_links):
            z = link_pitch * (link_idx + 0.5)
            _chain_link(upper, side_idx, link_idx, sx, z, link_height, zinc_chain)

    # --- Rigid standoff rungs (loop-emitted) ---
    for rung_idx in range(num_rungs):
        z = rung_spacing * (rung_idx + 1)
        # Main rung bar spanning between the two chains
        rung_span = 2.0 * chain_x + 0.040
        _cyl_x(upper, f"rung_{rung_idx}", rung_span, 0.013,
               (0.0, 0.0, z), rung_steel)
        # Standoff spacer stubs projecting toward the wall (−Y)
        for side_idx, sx in enumerate((-chain_x, chain_x)):
            _cyl_y(
                upper,
                f"standoff_{rung_idx}_{side_idx}",
                standoff_length,
                0.010,
                (sx, -standoff_length / 2.0, z),
                rung_steel,
            )
            # Small foot pad at standoff tip
            upper.visual(
                Box((0.028, 0.006, 0.036)),
                origin=Origin(xyz=(sx, -standoff_length - 0.003, z)),
                material=rung_steel,
                name=f"standoff_pad_{rung_idx}_{side_idx}",
            )

    # --- Top head bar (connects the two chains at the ladder head) ---
    _cyl_x(upper, "head_bar", 2.0 * chain_x + 0.08, 0.016,
           (0.0, 0.0, ladder_height), hook_steel)

    # --- Bottom foot bar ---
    _cyl_x(upper, "foot_bar", 2.0 * chain_x + 0.10, 0.016,
           (0.0, 0.0, 0.018), rung_steel)

    # --- Pivot boss on head bar (where the hook attaches) ---
    _cyl_z(upper, "pivot_boss", 0.040, 0.022,
           (0.0, 0.0, ladder_height + 0.020), hook_steel)

    # =====================================================================
    # Child part: sill_hook  (windowsill hook frame on REVOLUTE pivot)
    # =====================================================================
    sill_hook = model.part("sill_hook")

    # Sill plate — rests horizontally on top of the windowsill, above the
    # head bar and pivot boss.  At q=0 the hook is deployed: plate extends
    # in +Y (into the building interior) from the pivot.
    # Raised so its bottom (z=0.044) clears the pivot_boss top (z=0.040).
    sill_hook.visual(
        Box((0.28, 0.14, 0.012)),
        origin=Origin(xyz=(0.0, 0.07, 0.050)),
        material=hook_steel,
        name="sill_plate",
    )

    # Hook lip — hangs down on the interior side of the sill to prevent
    # the ladder from pulling away.
    sill_hook.visual(
        Box((0.26, 0.012, 0.14)),
        origin=Origin(xyz=(0.0, 0.134, -0.026)),
        material=hook_steel,
        name="hook_lip",
    )

    # Outer retention flange — short lip on the outside face to stop
    # the hook sliding off the sill outward.  Positioned to clear the
    # pivot boss (y < -0.022).
    sill_hook.visual(
        Box((0.26, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, -0.029, 0.014)),
        material=hook_steel,
        name="outer_flange",
    )

    # Pivot bracket ears — two plates that straddle the pivot boss.
    # Raised to clear the head bar (cylinder top at z=0.016).
    for ear_idx, ex in enumerate((-0.026, 0.026)):
        sill_hook.visual(
            Box((0.010, 0.050, 0.040)),
            origin=Origin(xyz=(ex, -0.025, 0.038)),
            material=hook_steel,
            name=f"pivot_ear_{ear_idx}",
        )

    # Pivot pin through the bracket and boss.
    _cyl_x(sill_hook, "pivot_pin", 0.072, 0.008,
           (0.0, -0.025, 0.038), dark_pin)

    # =====================================================================
    # Articulation: sill_hook_pivot  (REVOLUTE)
    # =====================================================================
    # Joint origin at the top of the ladder (pivot boss centre).
    # Axis along X so positive q folds the hook upward (+Y → +Z).
    # q=0: deployed (hook extends over sill).
    # q=upper: folded (hook swings up, flat against the ladder for storage).
    model.articulation(
        "sill_hook_pivot",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=sill_hook,
        origin=Origin(xyz=(0.0, 0.0, ladder_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.5,
            lower=0.0,
            upper=1.40,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_ladder")
    sill_hook = object_model.get_part("sill_hook")
    pivot = object_model.get_articulation("sill_hook_pivot")

    # --- Chain side-members exist and are loop-emitted ---
    chain_links = [v for v in upper.visuals if v.name.startswith("chain_link_")]
    ctx.check(
        "chain side-members present",
        len(chain_links) >= 48,
        details=(
            f"Expected at least 48 chain link visuals (24 per side × 2 sides), "
            f"got {len(chain_links)}."
        ),
    )

    # Verify both sides are present
    side_0_links = [v for v in chain_links if v.name.startswith("chain_link_0_")]
    side_1_links = [v for v in chain_links if v.name.startswith("chain_link_1_")]
    ctx.check(
        "both chain sides emitted",
        len(side_0_links) >= 24 and len(side_1_links) >= 24,
        details=f"side_0={len(side_0_links)}, side_1={len(side_1_links)}",
    )

    # --- Rigid standoff rungs exist ---
    rungs = [v for v in upper.visuals if v.name.startswith("rung_")]
    ctx.check(
        "rigid standoff rungs present",
        len(rungs) >= 12,
        details=f"Expected at least 12 rungs, got {len(rungs)}.",
    )

    # --- Standoff spacers present ---
    standoffs = [v for v in upper.visuals if v.name.startswith("standoff_")]
    ctx.check(
        "wall standoff spacers present",
        len(standoffs) >= 24,
        details=(
            f"Each rung should carry standoff stubs on both sides; "
            f"got {len(standoffs)} standoff visuals."
        ),
    )

    # --- No cage or wall-plate hardware from parent ---
    cage_visuals = [v for v in upper.visuals
                    if v.name.startswith("cage_") or v.name.startswith("wall_plate_")]
    ctx.check(
        "no cage or wall-plate hardware",
        len(cage_visuals) == 0,
        details="Roll-up ladder must not retain the fixed-ladder cage or wall plates.",
    )

    # --- sill_hook_pivot joint works ---
    # At q=0 the hook_lip hangs below the pivot; positive q folds it upward.
    # Measure the hook_lip AABB (not the part origin, which sits at the pivot).
    rest_aabb = ctx.part_element_world_aabb(sill_hook, elem="hook_lip")
    with ctx.pose({pivot: 1.2}):
        folded_aabb = ctx.part_element_world_aabb(sill_hook, elem="hook_lip")

    ctx.check(
        "sill_hook_pivot folds the hook",
        (rest_aabb is not None and folded_aabb is not None
         and folded_aabb[0][2] > rest_aabb[0][2] + 0.05),
        details=(
            f"Positive pivot angle should lift the hook lip upward; "
            f"rest_min_z={rest_aabb[0][2] if rest_aabb else None}, "
            f"folded_min_z={folded_aabb[0][2] if folded_aabb else None}."
        ),
    )

    # --- Hook part has the expected named features ---
    hook_visuals = {v.name for v in sill_hook.visuals}
    ctx.check(
        "sill hook has plate and lip",
        "sill_plate" in hook_visuals and "hook_lip" in hook_visuals,
        details=f"Hook visuals: {sorted(hook_visuals)}",
    )

    # --- Allow small intentional overlap between pivot ears and pivot boss ---
    for ear_idx in (0, 1):
        ctx.allow_overlap(
            upper,
            sill_hook,
            elem_a="pivot_boss",
            elem_b=f"pivot_ear_{ear_idx}",
            reason=(
                "The pivot bracket ears straddle the head-bar pivot boss to "
                "form a clevis-style pinned connection for the hook."
            ),
        )

    # Allow pivot pin embedding inside the pivot boss
    ctx.allow_overlap(
        upper,
        sill_hook,
        elem_a="pivot_boss",
        elem_b="pivot_pin",
        reason=(
            "The pivot pin is captured inside the pivot boss bore to form "
            "the revolute connection between the hook and the ladder head."
        ),
    )

    # Allow outer flange wrapping around the head bar
    ctx.allow_overlap(
        upper,
        sill_hook,
        elem_a="head_bar",
        elem_b="outer_flange",
        reason=(
            "The outer retention flange wraps around the head bar to keep "
            "the hook frame seated on the windowsill."
        ),
    )

    # Proof: pivot connection stays in contact
    ctx.expect_contact(
        upper,
        sill_hook,
        elem_a="pivot_boss",
        elem_b="pivot_ear_0",
        name="pivot boss contacts ear 0",
    )

    return ctx.report()


object_model = build_object_model()
