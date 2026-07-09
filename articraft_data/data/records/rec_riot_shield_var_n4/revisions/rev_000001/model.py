from __future__ import annotations

"""Quad-fold collapsible riot shield (G-FOLD style, four-panel accordion).

Four rectangular gray ballistic fabric panels joined edge-to-edge by full-width
fold hinges in an accordion chain.  The shield folds flat into four stacked
sections and unfolds into a full-height barrier.  Matte gray woven front face
with dark charcoal binding tape around all edges, a black rectangular logo
patch near the top of the front panel, a cast aluminum carry-handle bracket
bolted to the panel with four dome-head carriage bolts (open rectangular grip
window), and a black webbing pull strap along the bottom edge.

Primary articulation: three revolute fold hinges chaining four panels in
alternating accordion sense.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Overall dimensions (meters) -------------------------------------------
N_PANELS = 4  # multiplicity axis: quad-fold accordion

PANEL_W = 0.50   # panel width (Y)
PANEL_H = 0.55   # single panel height (Z)
PANEL_T = 0.030  # soft-armor panel thickness (X)
TRIM_W = 0.024   # edge binding tape width
TRIM_T = 0.037   # binding wraps slightly proud of both faces

HINGE_GAP = 0.040  # panel slabs stand off the fold axis; the fabric roll spans it
HINGE_Z = PANEL_H + HINGE_GAP  # fold line sits above the root panel top edge
SLEEVE_R = 0.042   # fat fabric fold roll wrapping the hinge line
BIND_H = 0.52      # binding tapes stop short of the fold line

# Articulation angles.
FOLD0 = 0.85                         # rest-pose tilt per hinge
FOLD_UPPER = math.pi - FOLD0         # deployed: child extends flat from parent

# Handle bracket (unchanged from parent)
PLATE_W = 0.21
PLATE_H = 0.15
PLATE_T = 0.016
FRAME_OUT_W = 0.15
FRAME_OUT_H = 0.10
FRAME_IN_W = 0.10
FRAME_IN_H = 0.05
FRAME_DEPTH = 0.048
HANDLE_Z = 0.27

BOLT_R = 0.010
BOLT_Y = 0.088
BOLT_Z = 0.060

# Companion variation: per-panel binding color (surface only)
BINDING_COLORS = [
    (0.17, 0.17, 0.18, 1.0),  # panel 0: standard charcoal
    (0.15, 0.16, 0.18, 1.0),  # panel 1: cool-shifted
    (0.18, 0.17, 0.16, 1.0),  # panel 2: warm-shifted
    (0.16, 0.18, 0.17, 1.0),  # panel 3: green-shifted
]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rounded_plate() -> cq.Workplane:
    """Cast aluminum mounting plate with rounded corners (handle part frame)."""
    plate = cq.Workplane("YZ").rect(PLATE_W, PLATE_H).extrude(PLATE_T)
    try:
        plate = plate.edges("|X").fillet(0.018)
    except ValueError:
        pass
    return plate


def _grip_frame() -> cq.Workplane:
    """Open rectangular grip loop standing proud of the plate."""
    frame = (
        cq.Workplane("YZ", origin=(PLATE_T - 0.002, 0.0, 0.0))
        .rect(FRAME_OUT_W, FRAME_OUT_H)
        .rect(FRAME_IN_W, FRAME_IN_H)
        .extrude(FRAME_DEPTH)
    )
    try:
        frame = frame.edges("|X").fillet(0.008)
    except ValueError:
        pass
    return frame


def _add_binding(
    part,
    height: float,
    z_center: float,
    end_z: float,
    material: str = "charcoal_binding",
) -> None:
    """Charcoal binding tape along both side edges plus the free end edge."""
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        part.visual(
            Box((TRIM_T, TRIM_W, height)),
            origin=Origin(xyz=(0.0, sign * (PANEL_W / 2.0 - TRIM_W / 2.0), z_center)),
            material=material,
            name=f"side_binding_{i}",
        )
    part.visual(
        Box((TRIM_T, PANEL_W, TRIM_W)),
        origin=Origin(xyz=(0.0, 0.0, end_z)),
        material=material,
        name="end_binding",
    )


def _build_panel(
    part,
    panel_idx: int,
    is_root: bool,
    is_last: bool,
    binding_material: str,
) -> None:
    """Shared helper: slab + edge binding + fabric fold sleeve for one panel."""
    if is_root:
        # Root panel: slab extends upward from origin
        part.visual(
            Box((PANEL_T, PANEL_W, PANEL_H)),
            origin=Origin(xyz=(0.0, 0.0, PANEL_H / 2.0)),
            material="gray_ballistic_fabric",
            name="front_slab",
        )
        _add_binding(part, BIND_H, BIND_H / 2.0, TRIM_W / 2.0, material=binding_material)
        if not is_last:
            # Hinge sleeve at the top fold line
            part.visual(
                Cylinder(radius=SLEEVE_R, length=PANEL_W),
                origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=binding_material,
                name="hinge_sleeve",
            )
    else:
        # Non-root panel: slab extends downward from the parent hinge frame
        part.visual(
            Box((PANEL_T, PANEL_W, PANEL_H)),
            origin=Origin(xyz=(0.0, 0.0, -(HINGE_GAP + PANEL_H / 2.0))),
            material="gray_ballistic_fabric",
            name="slab",
        )
        _add_binding(
            part,
            BIND_H,
            -(HINGE_GAP + PANEL_H - BIND_H / 2.0),
            -(HINGE_GAP + PANEL_H - TRIM_W / 2.0),
            material=binding_material,
        )
        if not is_last:
            # Hinge sleeve at the bottom fold line (opposite edge from parent hinge)
            sleeve_z = -(PANEL_H + 2.0 * HINGE_GAP)
            part.visual(
                Cylinder(radius=SLEEVE_R, length=PANEL_W),
                origin=Origin(xyz=(0.0, 0.0, sleeve_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=binding_material,
                name="hinge_sleeve",
            )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_riot_shield")

    # --- Materials ----------------------------------------------------------
    model.material("gray_ballistic_fabric", rgba=(0.60, 0.61, 0.62, 1.0))
    model.material("dark_ballistic_fabric", rgba=(0.11, 0.11, 0.12, 1.0))
    for idx in range(N_PANELS):
        model.material(f"charcoal_binding_{idx}", rgba=BINDING_COLORS[idx])
    model.material("patch_black", rgba=(0.04, 0.04, 0.045, 1.0))
    model.material("patch_white", rgba=(0.93, 0.93, 0.92, 1.0))
    model.material("cast_aluminum", rgba=(0.63, 0.64, 0.66, 1.0))
    model.material("bolt_steel", rgba=(0.24, 0.24, 0.26, 1.0))
    model.material("webbing_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("grommet_metal", rgba=(0.45, 0.45, 0.47, 1.0))

    # --- Emit panels in a loop (loop-emitted, not hand-written) -------------
    panels = []
    for i in range(N_PANELS):
        is_root = (i == 0)
        is_last = (i == N_PANELS - 1)
        binding_mat = f"charcoal_binding_{i}"
        panel = model.part(f"panel_{i}")
        _build_panel(panel, i, is_root, is_last, binding_mat)
        panels.append(panel)

    # --- Panel 0 extras: logo patch, pull strap, grommets -------------------
    front = panels[0]

    front.visual(
        Box((0.006, 0.20, 0.11)),
        origin=Origin(xyz=(PANEL_T / 2.0 + 0.002, 0.0, 0.435)),
        material="patch_black",
        name="logo_patch",
    )
    front.visual(
        Box((0.003, 0.15, 0.034)),
        origin=Origin(xyz=(PANEL_T / 2.0 + 0.0055, 0.0, 0.435)),
        material="patch_white",
        name="logo_lettering_bar",
    )
    front.visual(
        Box((0.012, 0.16, 0.022)),
        origin=Origin(xyz=(PANEL_T / 2.0 + 0.004, 0.0, 0.030)),
        material="webbing_black",
        name="bottom_pull_strap",
    )
    for g in range(3):
        front.visual(
            Cylinder(radius=0.008, length=PANEL_T + 0.010),
            origin=Origin(
                xyz=(0.0, -0.15 + 0.15 * g, TRIM_W / 2.0 + 0.010),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="grommet_metal",
            name=f"grommet_{g}",
        )

    # --- Carry-handle bracket (bolted rigid mount on panel_0 face) ----------
    handle = model.part("carry_handle")
    handle.visual(
        mesh_from_cadquery(_rounded_plate(), "handle_plate", tolerance=0.0012),
        material="cast_aluminum",
        name="mount_plate",
    )
    handle.visual(
        mesh_from_cadquery(_grip_frame(), "handle_grip_frame", tolerance=0.0012),
        material="cast_aluminum",
        name="grip_frame",
    )
    corners = ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))
    for ci, (sy, sz) in enumerate(corners):
        y = sy * BOLT_Y
        z = sz * BOLT_Z
        handle.visual(
            Cylinder(radius=BOLT_R, length=0.016),
            origin=Origin(xyz=(PLATE_T + 0.002, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="bolt_steel",
            name=f"bolt_shank_{ci}",
        )
        handle.visual(
            Sphere(radius=BOLT_R),
            origin=Origin(xyz=(PLATE_T + 0.010, y, z)),
            material="bolt_steel",
            name=f"bolt_dome_{ci}",
        )

    model.articulation(
        "panel_to_handle",
        ArticulationType.FIXED,
        parent=front,
        child=handle,
        origin=Origin(xyz=(PANEL_T / 2.0, 0.0, HANDLE_Z)),
    )

    # --- Accordion fold hinge chain (3 revolute hinges for 4 panels) --------
    for i in range(N_PANELS - 1):
        parent_panel = panels[i]
        child_panel = panels[i + 1]

        # Hinge origin: at the fold line between parent and child
        if i == 0:
            # Root panel: fold line is above the slab top
            hinge_z = HINGE_Z
        else:
            # Non-root panel: fold line is below the slab bottom
            hinge_z = -(PANEL_H + 2.0 * HINGE_GAP)

        # Accordion alternating sense: even hinges tilt back at rest,
        # odd hinges tilt forward, creating the zigzag rest pose.
        rpy_pitch = FOLD0 if (i % 2 == 0) else -FOLD0

        model.articulation(
            f"fold_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=parent_panel,
            child=child_panel,
            origin=Origin(xyz=(0.0, 0.0, hinge_z), rpy=(0.0, rpy_pitch, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=2.0,
                lower=-FOLD0,
                upper=FOLD_UPPER,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    handle = object_model.get_part("carry_handle")
    hinges = [object_model.get_articulation(f"fold_hinge_{i}") for i in range(N_PANELS - 1)]

    # --- Overlap allowances: hinge sleeves wrap the fold line ----------------
    for i in range(N_PANELS - 1):
        parent_name = f"panel_{i}"
        child_name = f"panel_{i + 1}"
        child_slab = "slab"  # all non-root panels use "slab"

        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a="hinge_sleeve",
            elem_b=child_slab,
            reason=(
                f"fold_hinge_{i}: fabric fold sleeve wraps the hinge line "
                f"and hugs the child panel top edge"
            ),
        )
        for j in range(2):
            ctx.allow_overlap(
                parent_name,
                child_name,
                elem_a="hinge_sleeve",
                elem_b=f"side_binding_{j}",
                reason=(
                    f"fold_hinge_{i}: fold sleeve fabric wraps over the "
                    f"child panel side binding at the hinge line"
                ),
            )

    # --- Prompt-specific presence checks -----------------------------------
    front = panels[0]

    ctx.check(
        "four_panels_present",
        all(p is not None for p in panels),
        "accordion must emit 4 panels: panel_0 through panel_3",
    )
    ctx.check(
        "three_fold_hinges_present",
        all(h is not None for h in hinges),
        "accordion must chain 4 panels with 3 fold hinges: fold_hinge_0..fold_hinge_2",
    )
    ctx.check(
        "logo_patch_present",
        front.get_visual("logo_patch") is not None,
        "panel_0 (front) must carry the black logo patch",
    )
    ctx.check(
        "grip_window_frame_present",
        handle.get_visual("grip_frame") is not None,
        "handle bracket must include the open grip frame",
    )
    ctx.check(
        "four_bolts_present",
        all(handle.get_visual(f"bolt_dome_{ci}") is not None for ci in range(4)),
        "handle bracket must be bolted with four dome-head bolts",
    )

    # Handle bracket mounted flush on panel_0 face.
    ctx.expect_contact(handle, front, elem_a="mount_plate", elem_b="front_slab")
    ctx.expect_within(
        handle, front, axes="yz",
        inner_elem="mount_plate", outer_elem="front_slab",
    )

    # Fold hinge continuity: each sleeve stays engaged with the child panel.
    for i in range(N_PANELS - 1):
        ctx.expect_contact(
            panels[i], panels[i + 1],
            elem_a="hinge_sleeve", elem_b="slab",
            name=f"fold_hinge_{i}_sleeve_contact",
        )

    # Each fold hinge has sufficient travel for the accordion mechanism.
    for i, hinge in enumerate(hinges):
        limits = hinge.motion_limits
        ctx.check(
            f"fold_hinge_{i}_travel",
            limits is not None and (limits.upper - limits.lower) > 2.5,
            f"fold_hinge_{i} must travel from nearly closed to fully deployed",
        )

    # --- Deployed pose: all panels form a continuous full-height barrier -----
    # Even hinge 0 at FOLD_UPPER (child_angle = pi, panel extends up)
    # Odd hinge 1 at FOLD0 (child_angle = 0, panel continues up from parent)
    # Even hinge 2 at -FOLD0 (child_angle = 0, panel continues up from parent)
    deployed = {}
    for i, hinge in enumerate(hinges):
        if i == 0:
            deployed[hinge] = FOLD_UPPER
        elif i % 2 == 1:
            deployed[hinge] = FOLD0
        else:
            deployed[hinge] = -FOLD0

    with ctx.pose(deployed):
        root_pos = ctx.part_world_position(panels[0])
        last_pos = ctx.part_world_position(panels[-1])
        ctx.check(
            "deployed_barrier_height",
            (root_pos is not None and last_pos is not None
             and last_pos[2] > root_pos[2] + 1.5),
            f"deployed 4-panel accordion barrier must exceed 1.5 m; "
            f"root_z={root_pos}, panel_3_z={last_pos}",
        )

    return ctx.report()


object_model = build_object_model()
