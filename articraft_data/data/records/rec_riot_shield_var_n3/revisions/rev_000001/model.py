from __future__ import annotations

"""Tri-fold soft-armor riot shield (G-FOLD style, three-panel accordion).

Three matte gray ballistic-fabric panels joined edge-to-edge by full-width
fabric fold hinges in an accordion Z-fold so the shield collapses into three
stacked sections for storage/transport and unfolds into a tall deployed barrier.
Each panel has dark charcoal binding tape around its edges.  Panel 0
(front/bottom) carries a black logo patch near the top, a cast aluminum
carry-handle bracket bolted on with four dome-head carriage bolts (open
rectangular grip window), a black webbing pull strap along the bottom edge,
and grommet eyelets.  Consecutive panels are chained with revolute fold
hinges whose fold direction alternates for accordion collapse.
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
N_PANELS = 3

PANEL_W = 0.50       # panel width (Y)
PANEL_H = 0.55       # single panel height (Z)
PANEL_T = 0.030      # soft-armor panel thickness (X)
TRIM_W = 0.024       # edge binding tape width
TRIM_T = 0.037       # binding wraps slightly proud of both faces

HINGE_GAP = 0.040    # panel slabs stand off the fold axis; the fabric roll spans it
HINGE_Z = PANEL_H + HINGE_GAP   # fold line sits above the root panel bottom edge
SLEEVE_R = 0.042     # fat fabric fold roll wrapping the hinge line
BIND_H = 0.52        # binding tapes stop short of the fold line

# Rest pose: child panels tilted forward by FOLD0; the three-panel chain
# clears itself at rest with the larger fold angle.
FOLD0 = 1.15
FOLD_LOWER = -0.62                   # fold nearly closed (panels almost face to face)
FOLD_UPPER = math.pi - FOLD0         # unfold flat: panel extends straight up

# Handle bracket
PLATE_W = 0.21       # Y
PLATE_H = 0.15       # Z
PLATE_T = 0.016      # X
FRAME_OUT_W = 0.15
FRAME_OUT_H = 0.10
FRAME_IN_W = 0.10
FRAME_IN_H = 0.05
FRAME_DEPTH = 0.048
HANDLE_Z = 0.27      # bracket center height on panel 0

BOLT_R = 0.010
BOLT_Y = 0.088
BOLT_Z = 0.060


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


def _add_binding(part, height: float, z_center: float, end_z: float) -> None:
    """Charcoal binding tape along both side edges plus the free end edge."""
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        part.visual(
            Box((TRIM_T, TRIM_W, height)),
            origin=Origin(xyz=(0.0, sign * (PANEL_W / 2.0 - TRIM_W / 2.0), z_center)),
            material="charcoal_binding",
            name=f"side_binding_{i}",
        )
    part.visual(
        Box((TRIM_T, PANEL_W, TRIM_W)),
        origin=Origin(xyz=(0.0, 0.0, end_z)),
        material="charcoal_binding",
        name="end_binding",
    )


def _add_panel_body(part, panel_index: int, n_panels: int) -> None:
    """Shared geometry helper: slab + edge bindings + fabric fold sleeve.

    Root panel (index 0) has its part frame at world z=0 with the slab
    extending upward.  Middle child panels (index > 0, not last) have their
    part frame at the parent hinge line with the slab extending downward; a
    fabric fold sleeve is added at their far edge for the next hinge.
    The last child panel has its slab extending upward from the hinge so that
    the chain of pi-rotation hinges stacks panels vertically when deployed.
    """
    is_root = panel_index == 0
    is_last = panel_index == n_panels - 1

    if is_root:
        slab_z = PANEL_H / 2.0
        bind_z_center = BIND_H / 2.0
        bind_end_z = TRIM_W / 2.0
        sleeve_z = HINGE_Z
    elif not is_last:
        slab_z = -(HINGE_GAP + PANEL_H / 2.0)
        bind_z_center = -(HINGE_GAP + PANEL_H - BIND_H / 2.0)
        bind_end_z = -(HINGE_GAP + PANEL_H - TRIM_W / 2.0)
        sleeve_z = -(PANEL_H + 2.0 * HINGE_GAP)
    else:
        # Last panel: slab extends UP from hinge for proper deployed stacking
        slab_z = HINGE_GAP + PANEL_H / 2.0
        bind_z_center = HINGE_GAP + PANEL_H - BIND_H / 2.0
        bind_end_z = HINGE_GAP + PANEL_H - TRIM_W / 2.0
        sleeve_z = None

    part.visual(
        Box((PANEL_T, PANEL_W, PANEL_H)),
        origin=Origin(xyz=(0.0, 0.0, slab_z)),
        material="gray_ballistic_fabric",
        name="slab",
    )
    _add_binding(part, BIND_H, bind_z_center, bind_end_z)

    if sleeve_z is not None:
        part.visual(
            Cylinder(radius=SLEEVE_R, length=PANEL_W),
            origin=Origin(xyz=(0.0, 0.0, sleeve_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="charcoal_binding",
            name="hinge_sleeve",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_riot_shield")

    model.material("gray_ballistic_fabric", rgba=(0.60, 0.61, 0.62, 1.0))
    model.material("charcoal_binding", rgba=(0.17, 0.17, 0.18, 1.0))
    model.material("patch_black", rgba=(0.04, 0.04, 0.045, 1.0))
    model.material("patch_white", rgba=(0.93, 0.93, 0.92, 1.0))
    model.material("cast_aluminum", rgba=(0.63, 0.64, 0.66, 1.0))
    model.material("bolt_steel", rgba=(0.24, 0.24, 0.26, 1.0))
    model.material("webbing_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("grommet_metal", rgba=(0.45, 0.45, 0.47, 1.0))

    # --- Emit panels in a loop with shared geometry helper -----------------
    panels = []
    for i in range(N_PANELS):
        panel = model.part(f"panel_{i}")
        panels.append(panel)
        _add_panel_body(panel, i, N_PANELS)

    # --- Panel 0 extras: logo patch, pull strap, grommets ------------------
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

    for i in range(3):
        front.visual(
            Cylinder(radius=0.008, length=PANEL_T + 0.010),
            origin=Origin(
                xyz=(0.0, -0.15 + 0.15 * i, TRIM_W / 2.0 + 0.010),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="grommet_metal",
            name=f"grommet_{i}",
        )

    # --- Carry-handle bracket (bolted rigid mount on panel 0 front face) ---
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
    for i, (sy, sz) in enumerate(corners):
        y = sy * BOLT_Y
        z = sz * BOLT_Z
        handle.visual(
            Cylinder(radius=BOLT_R, length=0.016),
            origin=Origin(xyz=(PLATE_T + 0.002, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="bolt_steel",
            name=f"bolt_shank_{i}",
        )
        handle.visual(
            Sphere(radius=BOLT_R),
            origin=Origin(xyz=(PLATE_T + 0.010, y, z)),
            material="bolt_steel",
            name=f"bolt_dome_{i}",
        )

    model.articulation(
        "panel_to_handle",
        ArticulationType.FIXED,
        parent=front,
        child=handle,
        origin=Origin(xyz=(PANEL_T / 2.0, 0.0, HANDLE_Z)),
    )

    # --- Fold hinges: chain consecutive panels with uniform joint policy ---
    # Both hinges share axis/limits; the Z-fold alternation arises from the
    # geometry convention (middle-panel slab down, last-panel slab up).
    for i in range(N_PANELS - 1):
        if i == 0:
            hinge_xyz = (0.0, 0.0, HINGE_Z)
        else:
            hinge_xyz = (0.0, 0.0, -(PANEL_H + 2.0 * HINGE_GAP))

        model.articulation(
            f"fold_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=panels[i],
            child=panels[i + 1],
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, FOLD0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=15.0, velocity=2.0,
                lower=FOLD_LOWER, upper=FOLD_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    handle = object_model.get_part("carry_handle")
    hinges = [
        object_model.get_articulation(f"fold_hinge_{i}")
        for i in range(N_PANELS - 1)
    ]

    # --- Overlap allowances: hinge sleeves wrap adjacent panel edges -------
    for i in range(N_PANELS - 1):
        parent_name = f"panel_{i}"
        child_name = f"panel_{i + 1}"
        ctx.allow_overlap(
            parent_name, child_name,
            elem_a="hinge_sleeve", elem_b="slab",
            reason="fabric fold sleeve wraps the hinge line and hugs the child panel edge",
        )
        for j in range(2):
            ctx.allow_overlap(
                parent_name, child_name,
                elem_a="hinge_sleeve", elem_b=f"side_binding_{j}",
                reason="fold sleeve fabric wraps over the child panel side binding at the hinge line",
            )

    # --- Presence checks ---------------------------------------------------
    ctx.check(
        "three_panels_exist",
        all(p is not None for p in panels),
        "shield must have three accordion-folded panels",
    )
    ctx.check(
        "two_fold_hinges_exist",
        all(h is not None for h in hinges),
        "three-panel accordion requires two fold hinges",
    )
    ctx.check(
        "logo_patch_on_panel_0",
        panels[0].get_visual("logo_patch") is not None,
        "panel_0 must carry the black logo patch",
    )
    ctx.check(
        "grip_window_frame_present",
        handle.get_visual("grip_frame") is not None,
        "handle bracket must include the open grip frame",
    )
    ctx.check(
        "four_bolts_present",
        all(handle.get_visual(f"bolt_dome_{i}") is not None for i in range(4)),
        "handle bracket must be bolted with four dome-head bolts",
    )

    # --- Handle bracket mounted flush on panel 0 front face ----------------
    ctx.expect_contact(handle, panels[0], elem_a="mount_plate", elem_b="slab")
    ctx.expect_within(
        handle, panels[0], axes="yz",
        inner_elem="mount_plate", outer_elem="slab",
    )

    # --- Hinge sleeve continuity at each fold line -------------------------
    for i in range(N_PANELS - 1):
        ctx.expect_contact(
            panels[i], panels[i + 1],
            elem_a="hinge_sleeve", elem_b="slab",
            name=f"fold_hinge_{i}_sleeve_contact",
        )

    # --- Fold travel range for each hinge ------------------------------------
    for i, hinge in enumerate(hinges):
        limits = hinge.motion_limits
        ctx.check(
            f"fold_hinge_{i}_travel_range",
            limits is not None and (limits.upper - limits.lower) > 2.5,
            f"fold_hinge_{i} must travel from nearly closed to fully deployed",
        )

    # --- Deployed pose: all panels extend into a tall barrier --------------
    deployed_pose = {h: FOLD_UPPER for h in hinges}
    with ctx.pose(deployed_pose):
        for i in range(N_PANELS - 1):
            ctx.expect_overlap(
                panels[i + 1], panels[i],
                axes="xy", min_overlap=0.02,
                elem_a="slab", elem_b="slab",
                name=f"deployed_panels_{i}_and_{i + 1}_coplanar",
            )

    # Verify deployed height: panel_2 origin is well above panel_0 origin.
    with ctx.pose(deployed_pose):
        p0_pos = ctx.part_world_position(panels[0])
        p2_pos = ctx.part_world_position(panels[-1])
        ctx.check(
            "deployed_shield_is_tall",
            p0_pos is not None and p2_pos is not None and p2_pos[2] > p0_pos[2] + 1.0,
            f"deployed panels should span over 1 m; p0={p0_pos}, p2={p2_pos}",
        )

    # --- Accordion alternation: at rest the panels form a Z-fold ----------
    # Panel_1 hangs behind panel_0 and panel_2 extends forward from panel_1.
    p1_pos = ctx.part_world_position(panels[1])
    p2_pos = ctx.part_world_position(panels[2])
    ctx.check(
        "accordion_z_fold_geometry",
        (p1_pos is not None and p2_pos is not None
         and p2_pos[2] < p1_pos[2]),
        f"Z-fold: panel_2 origin should be lower than panel_1 at rest; p1={p1_pos}, p2={p2_pos}",
    )

    return ctx.report()


object_model = build_object_model()
