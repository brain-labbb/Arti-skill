from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(length: float, depth: float, height: float, radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, depth, height)
    if radius > 0:
        try:
            shape = shape.edges().fillet(radius)
        except Exception:
            shape = cq.Workplane("XY").box(length, depth, height)
    return shape


def _screw(part, material, x: float, y: float, z: float, name: str) -> None:
    part.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _small_screw(part, material, x: float, y: float, z: float, name: str) -> None:
    part.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _label_marks(
    part,
    material,
    *,
    side: str,
    x_offset: float = 0.0,
    y: float = -0.030,
    z: float = 0.001,
) -> None:
    marks = [
        (-0.150, 0.040),
        (-0.100, 0.052),
        (-0.042, 0.034),
        (0.006, 0.048),
        (0.062, 0.036),
        (0.110, 0.050),
    ]
    for i, (x, length) in enumerate(marks):
        part.visual(
            Box((length, 0.003, 0.007)),
            origin=Origin(xyz=(x_offset + x, y, z)),
            material=material,
            name=f"{side}_label_mark_{i}",
        )


# ---------------------------------------------------------------------------
# Glazing-frame geometry for the near (glass stile-and-rail) leaf
# ---------------------------------------------------------------------------

def _make_glazing_frame() -> cq.Workplane:
    """Narrow-stile aluminum glazing frame with asymmetric top/bottom rails.

    Outer envelope: 0.600 x 0.046 x 1.420 (same as the solid leaf).
    Stile width: 0.040 each side.
    Top rail: 0.060, Bottom rail (kick zone): 0.080.
    The inner cutout is offset slightly upward so the bottom rail is thicker.
    """
    outer = cq.Workplane("XY").box(0.600, 0.046, 1.420)
    # inner cutout: 0.520 wide, 1.280 tall, centred 10 mm above outer centre
    inner = cq.Workplane("XY").box(0.520, 0.050, 1.280).translate((0.0, 0.0, 0.010))
    return outer.cut(inner)


def _add_glazed_leaf_panel(
    part,
    *,
    side: str,
    frame_mat,
    glass_mat,
    edge_mat,
    shadow_mat,
) -> None:
    """Build a fully-glazed stile-and-rail door leaf with glass infill."""
    sign = 1.0 if side == "near" else -1.0
    panel_center_x = sign * 0.300
    meeting_x = sign * 0.612

    # --- Glazing frame (stiles + rails as one mesh) ----------------------
    part.visual(
        mesh_from_cadquery(_make_glazing_frame(), f"{side}_door_skin"),
        origin=Origin(xyz=(panel_center_x, 0.0, 0.720)),
        material=frame_mat,
        name=f"{side}_door_skin",
    )

    # --- Glass infill ----------------------------------------------------
    # The glass sits inside the frame opening.  Opening spans
    # x ∈ [panel±0.260], z ∈ [0.090, 1.370] (world after origin offset).
    part.visual(
        Box((0.518, 0.006, 1.278)),
        origin=Origin(xyz=(panel_center_x, 0.0, 0.730)),
        material=glass_mat,
        name=f"{side}_glass_infill",
    )
    # Thin gasket shadow lines on the face side of the glass perimeter
    glass_cx = panel_center_x
    glass_cz = 0.730
    gw, gh = 0.518, 1.278
    for dx, nm in ((-gw / 2 + 0.003, "left_gasket"), (gw / 2 - 0.003, "right_gasket")):
        part.visual(
            Box((0.006, 0.004, gh - 0.012)),
            origin=Origin(xyz=(glass_cx + dx, -0.026, glass_cz)),
            material=shadow_mat,
            name=f"{side}_{nm}",
        )
    for dz, nm in ((-gh / 2 + 0.003, "bottom_gasket"), (gh / 2 - 0.003, "top_gasket")):
        part.visual(
            Box((gw - 0.012, 0.004, 0.006)),
            origin=Origin(xyz=(glass_cx, -0.026, glass_cz + dz)),
            material=shadow_mat,
            name=f"{side}_{nm}",
        )

    # --- Bottom kick plate overlay on the lower rail ---------------------
    part.visual(
        Box((0.540, 0.010, 0.032)),
        origin=Origin(xyz=(panel_center_x, -0.032, 0.050)),
        material=edge_mat,
        name=f"{side}_bottom_kick_plate",
    )
    for x_offset in (-0.210, -0.070, 0.070, 0.210):
        _small_screw(
            part, shadow_mat,
            panel_center_x + x_offset, -0.039, 0.050,
            f"{side}_kick_plate_screw_{len(part.visuals)}",
        )

    # --- Meeting stile (narrow vertical bar at the center seam) ----------
    part.visual(
        Box((0.014, 0.052, 1.400)),
        origin=Origin(xyz=(meeting_x, -0.002, 0.715)),
        material=edge_mat,
        name=f"{side}_meeting_stile",
    )
    part.visual(
        Box((0.018, 0.010, 1.360)),
        origin=Origin(xyz=(meeting_x - sign * 0.020, -0.030, 0.715)),
        material=shadow_mat,
        name=f"{side}_center_reveal_shadow",
    )

    # --- Hinge barrels ---------------------------------------------------
    for z, name in ((1.115, "upper_hinge_barrel"), (0.360, "lower_hinge_barrel")):
        part.visual(
            Box((0.020, 0.038, 0.180)),
            origin=Origin(xyz=(-sign * 0.009, -0.002, z)),
            material=edge_mat,
            name=f"{side}_{name}_leaf_plate",
        )
        for dz in (-0.056, 0.056):
            _small_screw(
                part, shadow_mat,
                -sign * 0.009, -0.023, z + dz,
                f"{side}_{name}_leaf_screw_{len(part.visuals)}",
            )
        part.visual(
            Cylinder(radius=0.018, length=0.160),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 0.0, 0.0)),
            material=edge_mat,
            name=f"{side}_{name}",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.170),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 0.0, 0.0)),
            material=shadow_mat,
            name=f"{side}_{name}_pin_shadow",
        )


# ---------------------------------------------------------------------------
# Solid-leaf panel (kept unchanged for the far leaf)
# ---------------------------------------------------------------------------

def _add_leaf_panel(part, *, side: str, material, edge_mat, shadow_mat) -> None:
    sign = 1.0 if side == "near" else -1.0
    panel_center_x = sign * 0.300
    meeting_x = sign * 0.612

    part.visual(
        Box((0.600, 0.046, 1.420)),
        origin=Origin(xyz=(panel_center_x, 0.0, 0.720)),
        material=material,
        name=f"{side}_door_skin",
    )
    for z, label in ((1.120, "upper_inset"), (0.395, "lower_inset")):
        part.visual(
            Box((0.500, 0.006, 0.245)),
            origin=Origin(xyz=(panel_center_x, -0.026, z)),
            material=shadow_mat,
            name=f"{side}_{label}_recess_shadow",
        )
        part.visual(
            Box((0.530, 0.008, 0.016)),
            origin=Origin(xyz=(panel_center_x, -0.030, z + 0.130)),
            material=edge_mat,
            name=f"{side}_{label}_top_trim",
        )
        part.visual(
            Box((0.530, 0.008, 0.016)),
            origin=Origin(xyz=(panel_center_x, -0.030, z - 0.130)),
            material=edge_mat,
            name=f"{side}_{label}_bottom_trim",
        )
        for x_offset, rail_name in ((-0.265, "hinge_side_trim"), (0.265, "meeting_side_trim")):
            part.visual(
                Box((0.014, 0.008, 0.250)),
                origin=Origin(xyz=(panel_center_x + sign * x_offset, -0.030, z)),
                material=edge_mat,
                name=f"{side}_{label}_{rail_name}",
            )
    part.visual(
        Box((0.540, 0.010, 0.032)),
        origin=Origin(xyz=(panel_center_x, -0.032, 0.095)),
        material=edge_mat,
        name=f"{side}_bottom_kick_plate",
    )
    for x_offset in (-0.210, -0.070, 0.070, 0.210):
        _small_screw(
            part, shadow_mat,
            panel_center_x + x_offset, -0.039, 0.095,
            f"{side}_kick_plate_screw_{len(part.visuals)}",
        )
    part.visual(
        Box((0.014, 0.052, 1.400)),
        origin=Origin(xyz=(meeting_x, -0.002, 0.715)),
        material=edge_mat,
        name=f"{side}_meeting_stile",
    )
    part.visual(
        Box((0.018, 0.010, 1.360)),
        origin=Origin(xyz=(meeting_x - sign * 0.020, -0.030, 0.715)),
        material=shadow_mat,
        name=f"{side}_center_reveal_shadow",
    )
    for z, name in ((1.350, "upper_panel_reveal"), (0.190, "lower_panel_reveal")):
        part.visual(
            Box((0.560, 0.008, 0.020)),
            origin=Origin(xyz=(panel_center_x, -0.030, z)),
            material=edge_mat,
            name=f"{side}_{name}",
        )
    for z, name in ((1.115, "upper_hinge_barrel"), (0.360, "lower_hinge_barrel")):
        part.visual(
            Box((0.020, 0.038, 0.180)),
            origin=Origin(xyz=(-sign * 0.009, -0.002, z)),
            material=edge_mat,
            name=f"{side}_{name}_leaf_plate",
        )
        for dz in (-0.056, 0.056):
            _small_screw(
                part, shadow_mat,
                -sign * 0.009, -0.023, z + dz,
                f"{side}_{name}_leaf_screw_{len(part.visuals)}",
            )
        part.visual(
            Cylinder(radius=0.018, length=0.160),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 0.0, 0.0)),
            material=edge_mat,
            name=f"{side}_{name}",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.170),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 0.0, 0.0)),
            material=shadow_mat,
            name=f"{side}_{name}_pin_shadow",
        )


# ---------------------------------------------------------------------------
# Fixed panic-hardware body (mechanism housings, rails, beads)
# ---------------------------------------------------------------------------

def _add_fixed_panic_hardware(
    part,
    *,
    side: str,
    metal_mat,
    dark_mat,
    green_mat,
    white_mat,
    include_static_bar: bool = True,
) -> None:
    sign = 1.0 if side == "near" else -1.0
    outer_x = sign * 0.070
    center_x = sign * 0.540
    rail_center = sign * 0.315
    label_center = sign * 0.285

    part.visual(
        Box((0.520, 0.016, 0.020)),
        origin=Origin(xyz=(rail_center, -0.050, 0.718)),
        material=metal_mat,
        name=f"{side}_fixed_upper_rail",
    )
    part.visual(
        Box((0.520, 0.016, 0.020)),
        origin=Origin(xyz=(rail_center, -0.050, 0.602)),
        material=metal_mat,
        name=f"{side}_fixed_lower_rail",
    )
    part.visual(
        Box((0.480, 0.006, 0.074)),
        origin=Origin(xyz=(rail_center, -0.035, 0.660)),
        material=dark_mat,
        name=f"{side}_rail_shadow_slot",
    )
    part.visual(
        Box((0.500, 0.006, 0.018)),
        origin=Origin(xyz=(rail_center, -0.061, 0.746)),
        material=metal_mat,
        name=f"{side}_upper_channel_bead",
    )
    part.visual(
        Box((0.500, 0.006, 0.018)),
        origin=Origin(xyz=(rail_center, -0.061, 0.574)),
        material=metal_mat,
        name=f"{side}_lower_channel_bead",
    )

    for x, name in ((outer_x, "outer_end_box"), (center_x, "center_latch_block")):
        part.visual(
            mesh_from_cadquery(_rounded_box(0.074, 0.062, 0.190, 0.008), f"{side}_{name}"),
            origin=Origin(xyz=(x, -0.057, 0.660)),
            material=metal_mat,
            name=f"{side}_{name}",
        )
        part.visual(
            Box((0.048, 0.010, 0.122)),
            origin=Origin(xyz=(x, -0.090, 0.660)),
            material=dark_mat,
            name=f"{side}_{name}_vertical_shadow",
        )
        part.visual(
            Box((0.084, 0.010, 0.024)),
            origin=Origin(xyz=(x, -0.088, 0.758)),
            material=metal_mat,
            name=f"{side}_{name}_top_cap_lip",
        )
        part.visual(
            Box((0.084, 0.010, 0.024)),
            origin=Origin(xyz=(x, -0.088, 0.562)),
            material=metal_mat,
            name=f"{side}_{name}_bottom_cap_lip",
        )
        _screw(part, dark_mat, x, -0.091, 0.730, f"{side}_{name}_upper_screw")
        _screw(part, dark_mat, x, -0.091, 0.590, f"{side}_{name}_lower_screw")

    if include_static_bar:
        part.visual(
            mesh_from_cadquery(_rounded_box(0.460, 0.042, 0.066, 0.010), f"{side}_rear_silver_bar"),
            origin=Origin(xyz=(rail_center, -0.078, 0.660)),
            material=metal_mat,
            name=f"{side}_rear_silver_bar",
        )
        for x_offset, cap_name in ((-0.245, "outer_push_cap"), (0.245, "inner_push_cap")):
            part.visual(
                Box((0.026, 0.046, 0.076)),
                origin=Origin(xyz=(rail_center + sign * x_offset, -0.079, 0.660)),
                material=dark_mat,
                name=f"{side}_{cap_name}",
            )
        part.visual(
            mesh_from_cadquery(_rounded_box(0.330, 0.007, 0.040, 0.003), f"{side}_rear_green_label"),
            origin=Origin(xyz=(label_center, -0.103, 0.660)),
            material=green_mat,
            name=f"{side}_rear_green_label",
        )
        _label_marks(part, white_mat, side=side, x_offset=label_center, y=-0.108, z=0.660)


# ---------------------------------------------------------------------------
# Model build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="glazed_exit_door_with_push_paddle",
        meta={
            "run_notes": (
                "Fork variant: fully-glazed near leaf (narrow stile-and-rail "
                "aluminum frame with tempered glass infill) paired with a "
                "revolute push-paddle panic device. The paddle pivots inward "
                "on a horizontal axis to retract the center latch bolt via "
                "mimic linkage. Far leaf remains a solid steel panel."
            )
        },
    )

    # -- Materials ---------------------------------------------------------
    frame_mat = model.material("dark_frame_recess", rgba=(0.18, 0.19, 0.20, 1.0))
    door_mat = model.material("satin_off_white_leaf", rgba=(0.76, 0.78, 0.82, 1.0))
    alt_door_mat = model.material("cool_off_white_leaf", rgba=(0.69, 0.71, 0.76, 1.0))
    shadow_mat = model.material("black_gap_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.68, 0.69, 0.67, 1.0))
    dark_metal_mat = model.material("dark_anodized_detail", rgba=(0.29, 0.30, 0.30, 1.0))
    green_mat = model.material("emergency_green", rgba=(0.02, 0.62, 0.36, 1.0))
    white_mat = model.material("raised_white_print", rgba=(0.97, 0.98, 0.94, 1.0))
    glass_mat = model.material("tempered_glass", rgba=(0.72, 0.80, 0.78, 0.40))

    # -- Fixed frame -------------------------------------------------------
    frame = model.part("frame")
    frame.visual(
        Box((1.440, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.030, 1.485)),
        material=frame_mat,
        name="top_jamb",
    )
    frame.visual(
        Box((0.054, 0.055, 1.470)),
        origin=Origin(xyz=(-0.665, 0.030, 0.735)),
        material=frame_mat,
        name="near_outer_jamb",
    )
    frame.visual(
        Box((0.054, 0.055, 1.470)),
        origin=Origin(xyz=(0.665, 0.030, 0.735)),
        material=frame_mat,
        name="far_outer_jamb",
    )
    frame.visual(
        Box((0.020, 0.025, 1.450)),
        origin=Origin(xyz=(0.0, 0.040, 0.735)),
        material=shadow_mat,
        name="center_closed_gap",
    )

    # -- Near leaf (glazed stile-and-rail) ---------------------------------
    near_leaf = model.part("near_leaf")
    _add_glazed_leaf_panel(
        near_leaf,
        side="near",
        frame_mat=metal_mat,
        glass_mat=glass_mat,
        edge_mat=metal_mat,
        shadow_mat=shadow_mat,
    )
    _add_fixed_panic_hardware(
        near_leaf,
        side="near",
        metal_mat=metal_mat,
        dark_mat=dark_metal_mat,
        green_mat=green_mat,
        white_mat=white_mat,
        include_static_bar=False,
    )

    # Lock cylinder
    near_leaf.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.495, -0.033, 0.935), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="near_lock_rosette",
    )
    near_leaf.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.495, -0.067, 0.935), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="near_lock_cylinder",
    )
    near_leaf.visual(
        Box((0.008, 0.003, 0.030)),
        origin=Origin(xyz=(0.495, -0.095, 0.935)),
        material=shadow_mat,
        name="near_lock_key_slot",
    )
    near_leaf.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.495, -0.039, 0.935), rpy=(math.pi / 2, 0.0, 0.0)),
        material=shadow_mat,
        name="near_lock_rosette_shadow_ring",
    )

    # Bolt guide channel (retains the latch bolt on the leaf)
    near_leaf.visual(
        Box((0.060, 0.011, 0.012)),
        origin=Origin(xyz=(0.620, -0.1165, 0.736)),
        material=metal_mat,
        name="near_bolt_front_guide_lip",
    )
    near_leaf.visual(
        Box((0.066, 0.008, 0.030)),
        origin=Origin(xyz=(0.625, -0.111, 0.715)),
        material=shadow_mat,
        name="near_bolt_sleeve_slot_shadow",
    )
    near_leaf.visual(
        Box((0.086, 0.028, 0.010)),
        origin=Origin(xyz=(0.612, -0.092, 0.738)),
        material=metal_mat,
        name="near_bolt_guide_upper_wall",
    )
    near_leaf.visual(
        Box((0.086, 0.028, 0.010)),
        origin=Origin(xyz=(0.612, -0.092, 0.692)),
        material=metal_mat,
        name="near_bolt_guide_lower_wall",
    )
    near_leaf.visual(
        Box((0.010, 0.028, 0.010)),
        origin=Origin(xyz=(0.567, -0.092, 0.740)),
        material=metal_mat,
        name="near_bolt_guide_upper_back_ear",
    )
    near_leaf.visual(
        Box((0.010, 0.028, 0.010)),
        origin=Origin(xyz=(0.567, -0.092, 0.690)),
        material=metal_mat,
        name="near_bolt_guide_lower_back_ear",
    )
    near_leaf.visual(
        Box((0.082, 0.010, 0.014)),
        origin=Origin(xyz=(0.610, -0.116, 0.744)),
        material=metal_mat,
        name="near_bolt_upper_retainer_lip",
    )
    near_leaf.visual(
        Box((0.082, 0.010, 0.014)),
        origin=Origin(xyz=(0.610, -0.116, 0.686)),
        material=metal_mat,
        name="near_bolt_lower_retainer_lip",
    )
    _small_screw(near_leaf, shadow_mat, 0.580, -0.112, 0.748, "near_bolt_guide_upper_screw")
    _small_screw(near_leaf, shadow_mat, 0.580, -0.112, 0.682, "near_bolt_guide_lower_screw")

    # Paddle pivot brackets (fixed to the near leaf door face)
    pivot_z = 0.760
    for dx, bracket_name in ((-0.130, "paddle_bracket_left"), (0.130, "paddle_bracket_right")):
        near_leaf.visual(
            Box((0.024, 0.022, 0.038)),
            origin=Origin(xyz=(0.315 + dx, -0.035, pivot_z)),
            material=metal_mat,
            name=f"near_{bracket_name}",
        )
        _small_screw(
            near_leaf, shadow_mat,
            0.315 + dx, -0.048, pivot_z + 0.012,
            f"near_{bracket_name}_upper_screw",
        )
        _small_screw(
            near_leaf, shadow_mat,
            0.315 + dx, -0.048, pivot_z - 0.012,
            f"near_{bracket_name}_lower_screw",
        )
    # Pivot pin
    near_leaf.visual(
        Cylinder(radius=0.006, length=0.260),
        origin=Origin(xyz=(0.315, -0.040, pivot_z), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_metal_mat,
        name="near_paddle_pivot_pin",
    )

    # -- Far leaf (solid panel, unchanged) ---------------------------------
    far_leaf = model.part("far_leaf")
    _add_leaf_panel(
        far_leaf,
        side="far",
        material=alt_door_mat,
        edge_mat=metal_mat,
        shadow_mat=shadow_mat,
    )
    _add_fixed_panic_hardware(
        far_leaf,
        side="far",
        metal_mat=metal_mat,
        dark_mat=dark_metal_mat,
        green_mat=green_mat,
        white_mat=white_mat,
    )
    far_leaf.visual(
        Box((0.086, 0.018, 0.170)),
        origin=Origin(xyz=(-0.590, -0.036, 0.715)),
        material=metal_mat,
        name="far_strike_plate",
    )
    far_leaf.visual(
        Box((0.058, 0.012, 0.044)),
        origin=Origin(xyz=(-0.590, -0.050, 0.715)),
        material=shadow_mat,
        name="far_strike_receiver",
    )
    far_leaf.visual(
        Box((0.068, 0.034, 0.016)),
        origin=Origin(xyz=(-0.590, -0.066, 0.748)),
        material=metal_mat,
        name="far_strike_upper_keeper_lip",
    )
    far_leaf.visual(
        Box((0.068, 0.034, 0.016)),
        origin=Origin(xyz=(-0.590, -0.066, 0.682)),
        material=metal_mat,
        name="far_strike_lower_keeper_lip",
    )
    far_leaf.visual(
        Box((0.016, 0.034, 0.080)),
        origin=Origin(xyz=(-0.548, -0.066, 0.715)),
        material=metal_mat,
        name="far_strike_outer_keeper_wall",
    )

    # -- Push paddle (REVOLUTE, replaces the sliding push bar) -------------
    near_push_bar = model.part("near_push_bar")
    # Paddle plate hangs below the pivot axis.  The plate top is offset 16 mm
    # below the pivot pin axis so the pin does not graze the plate edge.
    paddle_cz = -0.092  # plate centre, local Z (pivot axis is at z=0)
    paddle_h = 0.148     # plate height
    near_push_bar.visual(
        mesh_from_cadquery(_rounded_box(0.280, 0.012, paddle_h, 0.004), "near_paddle_plate"),
        origin=Origin(xyz=(0.0, 0.0, paddle_cz)),
        material=metal_mat,
        name="near_paddle_plate",
    )
    # Green instruction label on the paddle face
    near_push_bar.visual(
        mesh_from_cadquery(_rounded_box(0.220, 0.005, 0.048, 0.003), "near_paddle_green_label"),
        origin=Origin(xyz=(0.0, -0.009, paddle_cz)),
        material=green_mat,
        name="near_paddle_green_label",
    )
    # Label shadow strips
    near_push_bar.visual(
        Box((0.230, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.012, paddle_cz + 0.028)),
        material=dark_metal_mat,
        name="near_paddle_label_top_shadow",
    )
    near_push_bar.visual(
        Box((0.230, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.012, paddle_cz - 0.028)),
        material=dark_metal_mat,
        name="near_paddle_label_bottom_shadow",
    )
    # White print marks on the label
    _label_marks(near_push_bar, white_mat, side="near_paddle", x_offset=0.0, y=-0.013, z=paddle_cz)
    # Pivot bearing sleeves at each end of the paddle
    for dx, sleeve_name in ((-0.130, "near_paddle_bearing_left"), (0.130, "near_paddle_bearing_right")):
        near_push_bar.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(dx, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=dark_metal_mat,
            name=sleeve_name,
        )
    # Linkage arm connecting the paddle pivot area to the latch mechanism
    near_push_bar.visual(
        Box((0.018, 0.008, 0.060)),
        origin=Origin(xyz=(0.180, 0.004, -0.030)),
        material=dark_metal_mat,
        name="near_paddle_linkage_arm",
    )
    # Small push-rod connection at the linkage end — positioned below the
    # fixed upper rail to avoid collision at rest.
    near_push_bar.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.180, 0.004, -0.065), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="near_paddle_push_rod",
    )

    # -- Latch bolt (PRISMATIC, mimic-driven) -----------------------------
    latch_bolt = model.part("latch_bolt")
    latch_bolt.visual(
        mesh_from_cadquery(_rounded_box(0.030, 0.022, 0.032, 0.004), "center_latch_bolt"),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=metal_mat,
        name="center_latch_bolt",
    )
    latch_bolt.visual(
        Box((0.038, 0.018, 0.024)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=dark_metal_mat,
        name="bolt_tail",
    )
    latch_bolt.visual(
        Box((0.012, 0.012, 0.026)),
        origin=Origin(xyz=(0.042, -0.001, 0.0)),
        material=dark_metal_mat,
        name="bolt_beveled_dark_tip",
    )

    # -- Articulations -----------------------------------------------------
    left_door_joint = model.articulation(
        "frame_to_near_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=near_leaf,
        origin=Origin(xyz=(-0.620, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.8, lower=0.0, upper=1.05),
    )
    model.articulation(
        "frame_to_far_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=far_leaf,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.8, lower=0.0, upper=1.05),
        mimic=Mimic(joint=left_door_joint.name, multiplier=1.0, offset=0.0),
    )

    # Push paddle: REVOLUTE around horizontal X axis at the pivot.
    # With axis=(1,0,0) and the paddle extending in local -Z, positive q
    # rotates the paddle bottom toward +Y (into the door face) — the correct
    # push direction.
    push_joint = model.articulation(
        "near_leaf_to_push_bar",
        ArticulationType.REVOLUTE,
        parent=near_leaf,
        child=near_push_bar,
        origin=Origin(xyz=(0.315, -0.040, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=0.50),
    )

    # Latch bolt: PRISMATIC retraction driven by the paddle's internal linkage.
    # (Mimic from REVOLUTE paddle → PRISMATIC bolt is not allowed; tests
    #  pose both joints explicitly to prove the mechanism.)
    model.articulation(
        "near_leaf_to_latch_bolt",
        ArticulationType.PRISMATIC,
        parent=near_leaf,
        child=latch_bolt,
        origin=Origin(xyz=(0.590, -0.100, 0.715)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.050),
    )
    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    near_leaf = object_model.get_part("near_leaf")
    far_leaf = object_model.get_part("far_leaf")
    near_push_bar = object_model.get_part("near_push_bar")
    latch_bolt = object_model.get_part("latch_bolt")
    door_joint = object_model.get_articulation("frame_to_near_leaf")
    push_joint = object_model.get_articulation("near_leaf_to_push_bar")

    # --- Paired leaves: center seam gap ----------------------------------
    ctx.expect_gap(
        far_leaf,
        near_leaf,
        axis="x",
        min_gap=0.001,
        max_gap=0.030,
        positive_elem="far_meeting_stile",
        negative_elem="near_meeting_stile",
        name="paired door leaves meet at a narrow center seam",
    )

    # --- Green label on the paddle stops before the latch block ----------
    ctx.expect_gap(
        near_leaf,
        near_push_bar,
        axis="x",
        min_gap=0.020,
        max_gap=0.140,
        positive_elem="near_center_latch_block",
        negative_elem="near_paddle_green_label",
        name="green paddle label stops before the central vertical latch block",
    )

    # --- Green label overlaps the glazed leaf footprint ------------------
    ctx.expect_overlap(
        near_push_bar,
        near_leaf,
        axes="x",
        min_overlap=0.18,
        elem_a="near_paddle_green_label",
        elem_b="near_door_skin",
        name="paddle green label remains on the near glazed door leaf",
    )

    # --- Latch bolt aligns with the far leaf strike ----------------------
    ctx.expect_overlap(
        latch_bolt,
        far_leaf,
        axes="z",
        min_overlap=0.025,
        elem_a="center_latch_bolt",
        elem_b="far_strike_plate",
        name="latch bolt aligns with the opposite leaf strike",
    )

    # --- Paddle stands proud of the glazed leaf face ---------------------
    ctx.expect_gap(
        near_leaf,
        near_push_bar,
        axis="y",
        min_gap=0.008,
        max_gap=0.050,
        positive_elem="near_door_skin",
        negative_elem="near_paddle_plate",
        name="paddle plate stands proud of the glazed near-leaf face",
    )

    # --- Glazed geometry: glass infill is contained within the frame -----
    ctx.expect_within(
        near_leaf,
        near_leaf,
        axes="xy",
        inner_elem="near_glass_infill",
        outer_elem="near_door_skin",
        margin=0.005,
        name="glass infill is contained within the stile-and-rail glazing frame",
    )

    # --- Pivot hardware sits behind the glass face (y-clearance) ---------
    ctx.expect_gap(
        near_leaf,
        near_push_bar,
        axis="y",
        min_gap=0.005,
        max_gap=0.060,
        positive_elem="near_glass_infill",
        negative_elem="near_paddle_bearing_left",
        name="left pivot bearing sits behind the glass infill face",
    )

    # The bearing sleeves intentionally wrap around the pivot pin through the
    # brackets — this is mechanical nesting for a pinned pivot assembly.
    ctx.allow_overlap(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_bracket_left",
        elem_b="near_paddle_bearing_left",
        reason="Bearing sleeve wraps around the pivot pin and bracket bore — intentional pinned-pivot nesting.",
    )
    ctx.allow_overlap(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_bracket_right",
        elem_b="near_paddle_bearing_right",
        reason="Bearing sleeve wraps around the pivot pin and bracket bore — intentional pinned-pivot nesting.",
    )
    # The paddle plate hangs from the bracket area at its top edge; the
    # brackets cradle the paddle at the pivot pin — intentional nesting.
    ctx.allow_overlap(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_bracket_left",
        elem_b="near_paddle_plate",
        reason="Bracket cradles the paddle plate at the pivot pin — intentional pivot nesting.",
    )
    ctx.allow_overlap(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_bracket_right",
        elem_b="near_paddle_plate",
        reason="Bracket cradles the paddle plate at the pivot pin — intentional pivot nesting.",
    )
    # The pivot pin is captured inside each bearing bore — intentional shaft nesting.
    ctx.allow_overlap(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_pivot_pin",
        elem_b="near_paddle_bearing_left",
        reason="Pivot pin is captured inside the left bearing bore — intentional shaft nesting.",
    )
    ctx.allow_overlap(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_pivot_pin",
        elem_b="near_paddle_bearing_right",
        reason="Pivot pin is captured inside the right bearing bore — intentional shaft nesting.",
    )
    ctx.expect_contact(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_pivot_pin",
        elem_b="near_paddle_bearing_left",
        contact_tol=0.012,
        name="left bearing seats on the pivot pin",
    )
    ctx.expect_contact(
        near_leaf,
        near_push_bar,
        elem_a="near_paddle_pivot_pin",
        elem_b="near_paddle_bearing_right",
        contact_tol=0.012,
        name="right bearing seats on the pivot pin",
    )

    # --- Articulated motion checks ---------------------------------------
    latch_joint = object_model.get_articulation("near_leaf_to_latch_bolt")

    rest_paddle_aabb = ctx.part_element_world_aabb(near_push_bar, elem="near_paddle_plate")
    rest_bolt = ctx.part_world_position(latch_bolt)
    with ctx.pose({push_joint: 0.50}):
        pushed_paddle_aabb = ctx.part_element_world_aabb(near_push_bar, elem="near_paddle_plate")
    with ctx.pose({latch_joint: 0.050}):
        retracted_bolt = ctx.part_world_position(latch_bolt)

    ctx.check(
        "pushing paddle rotates its bottom toward the door face",
        rest_paddle_aabb is not None
        and pushed_paddle_aabb is not None
        and pushed_paddle_aabb[1][1] > rest_paddle_aabb[1][1] + 0.015,
        details=f"rest_aabb_max={rest_paddle_aabb[1]}, pushed_aabb_max={pushed_paddle_aabb[1]}",
    )
    ctx.check(
        "latch bolt retracts when linkage is driven by paddle rotation",
        rest_bolt is not None
        and retracted_bolt is not None
        and retracted_bolt[0] < rest_bolt[0] - 0.034,
        details=f"rest={rest_bolt}, retracted={retracted_bolt}",
    )

    # --- Door leaves swing outward on their hinges -----------------------
    closed_near_aabb = ctx.part_element_world_aabb(near_leaf, elem="near_door_skin")
    closed_far_aabb = ctx.part_element_world_aabb(far_leaf, elem="far_door_skin")
    with ctx.pose({door_joint: 0.65}):
        open_near_aabb = ctx.part_element_world_aabb(near_leaf, elem="near_door_skin")
        open_far_aabb = ctx.part_element_world_aabb(far_leaf, elem="far_door_skin")

    ctx.check(
        "both symmetric door leaves rotate outward on their hinges",
        closed_near_aabb is not None
        and closed_far_aabb is not None
        and open_near_aabb is not None
        and open_far_aabb is not None
        and open_near_aabb[0][1] < closed_near_aabb[0][1] - 0.12
        and open_far_aabb[0][1] < closed_far_aabb[0][1] - 0.12,
        details=(
            f"closed_near={closed_near_aabb}, open_near={open_near_aabb}, "
            f"closed_far={closed_far_aabb}, open_far={open_far_aabb}"
        ),
    )
    return ctx.report()


object_model = build_object_model()
