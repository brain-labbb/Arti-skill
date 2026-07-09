from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Roof pitch (radians). Ridge is at u=0 and high; the slope falls toward the eave at +u.
PITCH = 0.42
_COS_P = math.cos(PITCH)
_SIN_P = math.sin(PITCH)
RIDGE_HEIGHT = 2.18

HALF_W = 0.72                    # half width across the slope (world Y)
EAVE_U = 2.60                    # down-slope length to the eave
VENT_U0, VENT_U1 = 0.07, 1.28   # vent opening band along the slope
VENT_HALF = 0.46                # vent half width across

# Prismatic sliding travel (companion variation 3 of 5: 0.15 / 0.30 / 0.50 / 0.70 / 0.90).
SLIDE_TRAVEL = 0.50

# Rail extent: covers the vent opening plus full travel plus small margins.
RAIL_U0 = VENT_U0 - 0.02
RAIL_U1 = VENT_U1 + SLIDE_TRAVEL + 0.02
RAIL_LEN = RAIL_U1 - RAIL_U0
RAIL_MID = (RAIL_U0 + RAIL_U1) / 2.0

# Height of the panel seating plane above the roof structural plane.
PANEL_W = 0.060


def _plane_xyz(u: float, v: float, w: float) -> tuple[float, float, float]:
    """Map roof-plane coords (u=down-slope, v=across=world Y, w=out-of-plane) to world xyz."""
    x = u * _COS_P + w * _SIN_P
    y = v
    z = RIDGE_HEIGHT - u * _SIN_P + w * _COS_P
    return (x, y, z)


def _plane_box(part, name, size, u, v, w, material):
    """Add a box lying in the pitched roof plane. size = (along-slope, across, out-of-plane)."""
    part.visual(
        Box(size),
        origin=Origin(xyz=_plane_xyz(u, v, w), rpy=(0.0, PITCH, 0.0)),
        material=material,
        name=name,
    )


def _plane_cyl_y(part, name, radius, length, u, v, w, material):
    """Add a Y-axis cylinder (runs across-slope, along the ridge direction) at a plane point."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_plane_xyz(u, v, w), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _vert_twine(part, name, top_xyz, length, radius, material):
    """A thin cord hanging straight down in world Z from a roof point."""
    cx, cy, cz = top_xyz
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(cx, cy, cz - length / 2.0)),
        material=material,
        name=name,
    )


def _add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cyl_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_greenhouse_vent_roof",
        meta={
            "class": "Agricultural/Greenhouse vent roof",
            "description": (
                "A pitched greenhouse roof bay: glazed pane grid in weathered aluminum and "
                "aged-steel glazing bars, a sliding vent panel on aluminum guide rails with "
                "prismatic down-slope travel, a latch handle, hanging tomato twine and a "
                "tensioned support wire."
            ),
            "companion_variations": [
                {"travel_m": 0.15, "label": "micro_vent"},
                {"travel_m": 0.30, "label": "light_vent"},
                {"travel_m": 0.50, "label": "standard_vent"},
                {"travel_m": 0.70, "label": "wide_vent"},
                {"travel_m": 0.90, "label": "full_vent"},
            ],
        },
    )

    aluminum = model.material("weathered_aluminum", rgba=(0.74, 0.75, 0.70, 1.0))
    aged_steel = model.material("aged_glazing_bar", rgba=(0.30, 0.26, 0.20, 1.0))
    galvanized = model.material("galvanized_hardware", rgba=(0.55, 0.57, 0.52, 1.0))
    glass = model.material("clear_sky_glass", rgba=(0.60, 0.80, 0.95, 0.26))
    rubber = model.material("black_epdm_seal", rgba=(0.02, 0.02, 0.02, 1.0))
    black_steel = model.material("black_painted_steel", rgba=(0.04, 0.04, 0.045, 1.0))
    bolt_dark = model.material("dark_fasteners", rgba=(0.03, 0.03, 0.03, 1.0))
    jute = model.material("jute_twine", rgba=(0.42, 0.34, 0.22, 1.0))
    wire = model.material("tension_wire", rgba=(0.62, 0.60, 0.55, 1.0))

    # ----------------------------------------------------------------- roof_frame
    roof = model.part("roof_frame")

    # Perimeter and structural rails of the pitched roof bay.
    _plane_box(roof, "ridge_rail", (0.085, 2 * HALF_W + 0.06, 0.060), 0.0, 0.0, 0.028, aluminum)
    _plane_box(roof, "eave_rail", (0.060, 2 * HALF_W + 0.06, 0.050), EAVE_U, 0.0, 0.022, aluminum)
    _plane_box(roof, "rake_rail_0", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, -HALF_W, 0.022, aluminum)
    _plane_box(roof, "rake_rail_1", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, HALF_W, 0.022, aged_steel)

    # Transoms (across-slope glazing bars) and mullions (down-slope bars) -> pane grid.
    _plane_box(roof, "transom_mid", (0.050, 2 * HALF_W, 0.044), 1.42, 0.0, 0.024, aged_steel)
    _plane_box(roof, "transom_low", (0.050, 2 * HALF_W, 0.044), 2.02, 0.0, 0.024, aluminum)
    for i, v in enumerate((-0.24, 0.24)):
        _plane_box(roof, f"mullion_lower_{i}", (1.22, 0.042, 0.040), 2.01, v, 0.026, aluminum)
    # Side mullions flanking the vent opening (span from ridge down past the curb sill).
    for i, v in enumerate((-0.60, 0.60)):
        _plane_box(roof, f"mullion_side_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Curb framing the vent opening (sill at the bottom, jambs down the sides).
    _plane_box(roof, "vent_curb_sill", (0.065, 2 * VENT_HALF + 0.12, 0.048), VENT_U1, 0.0, 0.024, aluminum)
    for i, v in enumerate((-VENT_HALF - 0.04, VENT_HALF + 0.04)):
        _plane_box(roof, f"vent_curb_jamb_{i}", (VENT_U1 - VENT_U0, 0.045, 0.046), (VENT_U0 + VENT_U1) / 2.0, v, 0.024, aluminum)

    # Header bar at top of vent opening (leading-edge stop for the sliding panel).
    _plane_box(roof, "vent_header", (0.055, 2 * VENT_HALF + 0.10, 0.046), VENT_U0, 0.0, 0.024, aluminum)

    # EPDM weather seals: header seal at top, sill seal at bottom, side seals along jambs.
    _plane_box(roof, "header_weather_seal", (0.022, 2 * VENT_HALF, 0.014), VENT_U0 + 0.02, 0.0, 0.048, rubber)
    _plane_box(roof, "sill_weather_seal", (0.024, 2 * VENT_HALF, 0.014), VENT_U1 - 0.02, 0.0, 0.046, rubber)
    for i, v in enumerate((-VENT_HALF - 0.01, VENT_HALF + 0.01)):
        _plane_box(roof, f"jamb_weather_seal_{i}", (VENT_U1 - VENT_U0 - 0.06, 0.012, 0.014), (VENT_U0 + VENT_U1) / 2.0, v, 0.048, rubber)

    # Fixed glazing panes around the vent (lower field + side strips). Thin clear sheets.
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((1.72, 2.31)):
            _plane_box(roof, f"glass_lower_{ci}_{ri}", (0.55, 0.44, 0.006), u, v, 0.045, glass)
    for i, v in enumerate((-0.61, 0.61)):
        _plane_box(roof, f"glass_side_{i}", (1.18, 0.18, 0.006), 0.71, v, 0.045, glass)

    # --- Sliding vent guide rails (vent_rail_0/1): fixed to roof frame ---
    # Each rail is an aluminum C-channel running down-slope, on either side of the vent opening.
    for i, v in enumerate((-VENT_HALF - 0.025, VENT_HALF + 0.025)):
        # Rail base (flat mounting bar on the roof frame)
        _plane_box(roof, f"vent_rail_{i}", (RAIL_LEN, 0.042, 0.015), RAIL_MID, v, 0.048, aluminum)
        # Rail guide lip (inner edge, forming the C-channel track)
        lip_offset = 0.017 * (1 if i == 0 else -1)
        _plane_box(roof, f"vent_rail_lip_{i}", (RAIL_LEN, 0.008, 0.024), RAIL_MID, v + lip_offset, 0.058, aluminum)
        # Top end stop (prevents panel from sliding past the header)
        _plane_box(roof, f"vent_rail_stop_top_{i}", (0.022, 0.044, 0.032), RAIL_U0 + 0.012, v, 0.058, galvanized)
        # Bottom end stop (prevents panel from sliding off the rails)
        _plane_box(roof, f"vent_rail_stop_bot_{i}", (0.022, 0.044, 0.032), RAIL_U1 - 0.012, v, 0.058, galvanized)

    # Rail mounting bolts (attaching rails to the mullions and curb frame).
    sx = 0
    for u, v in ((1.42, -0.40), (1.42, 0.40), (2.02, -0.40), (2.02, 0.40), (2.01, 0.0)):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_plane_xyz(u, v, 0.046), rpy=(0.0, PITCH, 0.0)),
            material=bolt_dark,
            name=f"roof_screw_{sx}",
        )
        sx += 1
    # Rail-specific fasteners along the guide rails.
    for u_off in (0.15, 0.55, 0.95, 1.35):
        for i, v in enumerate((-VENT_HALF - 0.025, VENT_HALF + 0.025)):
            roof.visual(
                Cylinder(radius=0.006, length=0.007),
                origin=Origin(xyz=_plane_xyz(RAIL_U0 + u_off, v, 0.058), rpy=(0.0, PITCH, 0.0)),
                material=bolt_dark,
                name=f"rail_bolt_{sx}",
            )
            sx += 1

    # Tensioned support wire spanning the bay in the roof plane (ends bear on the rake rails).
    _plane_box(roof, "support_wire", (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire)
    # Hanging tomato twine dropping through the wire, each with a tied loop knot at the bottom.
    for i, v in enumerate((-0.38, -0.05, 0.22, 0.50)):
        top = _plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.05 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}", (0.020, 0.014, 0.024), (top[0], top[1], top[2] - length), jute)

    # --------------------------------------------------------------- sliding_vent_panel
    # Part frame coincides with the articulation frame at q=0 (top of vent opening).
    # Local +X = down-slope, +Y = across slope, +Z = outward from roof plane.
    panel = model.part("sliding_vent_panel")
    SASH_LEN = VENT_U1 - VENT_U0           # 1.21
    SASH_HALF = VENT_HALF                  # 0.46
    midx = SASH_LEN / 2.0

    # Frame reference: bottom of sash frame at local z ≈ -0.005, top at z ≈ 0.033.
    frame_z = 0.014  # center of 0.038-thick frame

    # Glass pane (recessed slightly into the frame).
    _add_box(panel, "vent_glass", (SASH_LEN - 0.10, 2 * SASH_HALF - 0.08, 0.007), (midx, 0.0, 0.004), glass)

    # Sash frame rails (top = leading edge near ridge, bottom = trailing edge near eave).
    _add_box(panel, "sash_top_rail", (0.060, 2 * SASH_HALF, 0.038), (0.030, 0.0, frame_z), aluminum)
    _add_box(panel, "sash_bottom_rail", (0.080, 2 * SASH_HALF, 0.038), (SASH_LEN - 0.030, 0.0, frame_z), aluminum)

    # Stiles (side members of the sash frame).
    _add_box(panel, "sash_stile_0", (SASH_LEN, 0.052, 0.038), (midx, -SASH_HALF + 0.026, frame_z), aluminum)
    _add_box(panel, "sash_stile_1", (SASH_LEN, 0.052, 0.038), (midx, SASH_HALF - 0.026, frame_z), aged_steel)

    # Central glazing bar (supports the glass pane mid-span).
    _add_box(panel, "sash_glazing_bar", (0.038, 2 * SASH_HALF - 0.08, 0.028), (midx, 0.0, 0.024), aluminum)

    # Drip lip at the trailing (bottom) edge.
    _add_box(panel, "sash_drip_lip", (0.030, 2 * SASH_HALF - 0.06, 0.022), (SASH_LEN - 0.060, 0.0, -0.016), aluminum)

    # EPDM gaskets around the sash perimeter (bed against curb seals when closed).
    _add_box(panel, "sash_gasket_0", (SASH_LEN - 0.10, 0.016, 0.012), (midx, -SASH_HALF + 0.012, -0.008), rubber)
    _add_box(panel, "sash_gasket_1", (SASH_LEN - 0.10, 0.016, 0.012), (midx, SASH_HALF - 0.012, -0.008), rubber)
    _add_box(panel, "sash_gasket_bottom", (0.018, 2 * SASH_HALF - 0.06, 0.012), (SASH_LEN - 0.020, 0.0, 0.000), rubber)
    _add_box(panel, "sash_gasket_top", (0.018, 2 * SASH_HALF - 0.06, 0.012), (0.020, 0.0, 0.000), rubber)

    # Slider shoes (4 total, 2 per side) that engage the C-channel guide rails.
    # These are small galvanized blocks that ride in the rail track.
    for side_idx, yv in enumerate((-SASH_HALF - 0.003, SASH_HALF + 0.003)):
        for end_idx, xu in enumerate((0.10, SASH_LEN - 0.10)):
            panel.visual(
                Box((0.060, 0.032, 0.020)),
                origin=Origin(xyz=(xu, yv, -0.005)),
                material=galvanized,
                name=f"slider_shoe_{side_idx}_{end_idx}",
            )

    # Corner gusset plates at the four sash frame corners.
    for i, (x, v) in enumerate(((0.10, -0.40), (0.10, 0.40), (SASH_LEN - 0.06, -0.40), (SASH_LEN - 0.06, 0.40))):
        _add_box(panel, f"sash_corner_plate_{i}", (0.075, 0.045, 0.006), (x, v, 0.030), galvanized)

    # Fastening screws along the stiles and rails.
    for i, x in enumerate((0.22, 0.60, 0.98)):
        for j, v in enumerate((-0.41, 0.41)):
            panel.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, v, 0.034)),
                material=bolt_dark,
                name=f"sash_screw_{i}_{j}",
            )

    # Pull handle on the bottom rail (for manual sliding operation).
    # Mounted directly on top of the bottom rail, connected through X/Z overlap.
    _add_box(panel, "slide_pull_handle", (0.120, 0.030, 0.024), (SASH_LEN - 0.10, 0.0, 0.035), black_steel)
    _add_box(panel, "slide_pull_grip", (0.100, 0.042, 0.016), (SASH_LEN - 0.10, 0.0, 0.050), rubber)

    # --------------------------------------------------------------- latch_handle
    # Compact sliding-panel latch: pivots on a Y-axis pin above the bottom rail,
    # hook tongue catches a keeper on the curb sill; handle extends upward for operation.
    latch = model.part("latch_handle")
    _add_cyl_y(latch, "latch_pivot_pin", 0.012, 0.065, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(latch, "latch_back_plate", (0.055, 0.075, 0.010), (0.0, 0.0, 0.008), galvanized)
    _add_box(latch, "latch_hook_tongue", (0.075, 0.024, 0.012), (0.055, 0.0, -0.018), black_steel)
    _add_box(latch, "latch_hook_drop", (0.018, 0.024, 0.030), (0.022, 0.0, -0.012), black_steel)
    _add_box(latch, "pull_handle_stem", (0.020, 0.028, 0.055), (-0.012, 0.0, 0.036), black_steel)
    _add_box(latch, "rubber_grip", (0.038, 0.100, 0.024), (-0.012, 0.0, 0.066), rubber)

    # ----------------------------------------------------------------- joints
    # Primary articulation: sliding vent panel translates DOWN-SLOPE along the rails.
    # Origin at the top of the vent opening (header/leading-edge seating plane).
    # The articulation frame is pitched with the roof; axis = (1, 0, 0) in that frame
    # means +q slides the panel down-slope, exposing the ridge-side vent opening.
    slide_origin = _plane_xyz(VENT_U0, 0.0, PANEL_W)
    slide_joint = model.articulation(
        "roof_to_sliding_vent",
        ArticulationType.PRISMATIC,
        parent=roof,
        child=panel,
        origin=Origin(xyz=slide_origin, rpy=(0.0, PITCH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.15, lower=0.0, upper=SLIDE_TRAVEL),
    )
    slide_joint.meta["role"] = "primary sliding vent panel on roof guide rails"

    # Latch swings on the sliding panel bottom rail (offset in Y to clear the pull handle).
    model.articulation(
        "panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(SASH_LEN - 0.03, 0.25, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")
    panel = object_model.get_part("sliding_vent_panel")
    latch = object_model.get_part("latch_handle")
    slide = object_model.get_articulation("roof_to_sliding_vent")
    latch_joint = object_model.get_articulation("panel_to_latch")

    # --- Intentional overlap allowances ---

    # Slider shoes engage the rail C-channel tracks (small local overlap).
    for side_idx in range(2):
        for end_idx in range(2):
            rail_name = f"vent_rail_{side_idx}"
            lip_name = f"vent_rail_lip_{side_idx}"
            shoe_name = f"slider_shoe_{side_idx}_{end_idx}"
            ctx.allow_overlap(
                "sliding_vent_panel",
                "roof_frame",
                elem_a=shoe_name,
                elem_b=rail_name,
                reason=f"Slider shoe {shoe_name} rides inside the vent_rail_{side_idx} C-channel track.",
            )
            ctx.allow_overlap(
                "sliding_vent_panel",
                "roof_frame",
                elem_a=shoe_name,
                elem_b=lip_name,
                reason=f"Slider shoe {shoe_name} is captured by the vent_rail_lip_{side_idx} guide edge.",
            )

    # Panel stile/rail seating onto the curb and header when closed.
    for stile in ("sash_stile_0", "sash_stile_1"):
        ctx.allow_overlap(
            "sliding_vent_panel",
            "roof_frame",
            elem_a=stile,
            elem_b="vent_header",
            reason="Closed panel leading edge seats against the vent header bar.",
        )
    ctx.allow_overlap(
        "sliding_vent_panel",
        "roof_frame",
        elem_a="sash_bottom_rail",
        elem_b="vent_curb_sill",
        reason="Closed panel trailing edge seats onto the curb sill.",
    )

    # Compliant EPDM weather seals are bedded (compressed) by the closed panel frame.
    for seal, members in (
        (
            "header_weather_seal",
            ("sash_top_rail", "sash_gasket_top", "sash_stile_0", "sash_stile_1"),
        ),
        (
            "sill_weather_seal",
            ("sash_bottom_rail", "sash_gasket_bottom", "sash_stile_0", "sash_stile_1"),
        ),
    ):
        for member in members:
            ctx.allow_overlap(
                "roof_frame",
                "sliding_vent_panel",
                elem_a=seal,
                elem_b=member,
                reason="Closed panel frame beds and compresses the fixed EPDM weather seal.",
            )
    for i in range(2):
        for member in ("sash_gasket_0", "sash_gasket_1", "sash_stile_0", "sash_stile_1"):
            ctx.allow_overlap(
                "roof_frame",
                "sliding_vent_panel",
                elem_a=f"jamb_weather_seal_{i}",
                elem_b=member,
                reason="Closed panel side edges bed against the jamb EPDM weather seal.",
            )

    # Latch hardware is captured at the panel bottom rail it mounts to.
    for latch_elem in ("latch_pivot_pin", "latch_back_plate", "latch_hook_drop", "latch_hook_tongue"):
        ctx.allow_overlap(
            "latch_handle",
            "sliding_vent_panel",
            elem_a=latch_elem,
            elem_b="sash_bottom_rail",
            reason="Latch hardware is captured at the panel bottom rail it mounts to.",
        )

    # Jamb weather seals pass through the slider shoes (both at panel side edge Y).
    for i in range(2):
        for side_idx in range(2):
            for end_idx in range(2):
                ctx.allow_overlap(
                    "roof_frame",
                    "sliding_vent_panel",
                    elem_a=f"jamb_weather_seal_{i}",
                    elem_b=f"slider_shoe_{side_idx}_{end_idx}",
                    reason="Jamb weather seal runs along the panel side edge where slider shoes also mount.",
                )

    # Pull handle is seated directly on the bottom rail surface.
    ctx.allow_overlap(
        "sliding_vent_panel",
        "sliding_vent_panel",
        elem_a="slide_pull_handle",
        elem_b="sash_bottom_rail",
        reason="Pull handle is surface-mounted on top of the panel bottom rail.",
    )

    # --- Classification and structure ---
    ctx.check(
        "classified as greenhouse vent roof",
        object_model.name == "agricultural_greenhouse_vent_roof"
        and object_model.meta.get("class") == "Agricultural/Greenhouse vent roof",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "has sliding_vent_panel, vent_rail_0/1 on roof_frame, and prismatic joint",
        panel is not None
        and roof is not None
        and latch is not None
        and slide is not None
        and roof.get_visual("vent_rail_0") is not None
        and roof.get_visual("vent_rail_1") is not None,
        details="Expected sliding_vent_panel part, vent_rail_0/1 visuals on roof_frame, and roof_to_sliding_vent joint.",
    )

    # The slide joint is prismatic and the axis drives down-slope translation.
    ctx.check(
        "roof_to_sliding_vent is PRISMATIC",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={slide.articulation_type}",
    )

    # --- Primary mechanism: prismatic sliding ---
    closed_pos = ctx.part_world_position(panel)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        open_pos = ctx.part_world_position(panel)

    # The panel must actually translate down-slope when the joint opens.
    ctx.check(
        "sliding_vent_panel translates down-slope when opened",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] > closed_pos[0] + 0.10,
        details=f"closed_x={closed_pos[0]:.3f}, open_x={open_pos[0]:.3f}",
    )

    # The panel follows the roof pitch: the Z drop matches travel * sin(pitch) ± tolerance.
    # This proves the panel stays in the roof plane (slides along the slope, not lifts off).
    expected_dz = SLIDE_TRAVEL * _SIN_P
    actual_dz = closed_pos[2] - open_pos[2] if (closed_pos and open_pos) else 0.0
    ctx.check(
        "sliding_vent_panel follows the roof pitch (stays in-plane)",
        closed_pos is not None
        and open_pos is not None
        and abs(actual_dz - expected_dz) < 0.04,
        details=f"expected_dz={expected_dz:.3f}, actual_dz={actual_dz:.3f}",
    )

    # The vent_rail_0 and vent_rail_1 are fixed to the roof and span the travel range.
    ctx.expect_overlap(
        panel,
        roof,
        axes="x",
        elem_a="sash_stile_0",
        elem_b="vent_rail_0",
        min_overlap=0.20,
        name="panel stile engages vent_rail_0 along the slide axis",
    )
    ctx.expect_overlap(
        panel,
        roof,
        axes="x",
        elem_a="sash_stile_1",
        elem_b="vent_rail_1",
        min_overlap=0.20,
        name="panel stile engages vent_rail_1 along the slide axis",
    )

    # At full travel, the panel still overlaps the rails (retained insertion).
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_overlap(
            panel,
            roof,
            axes="x",
            elem_a="sash_stile_0",
            elem_b="vent_rail_0",
            min_overlap=0.10,
            name="panel retains engagement with vent_rail_0 at full travel",
        )
        ctx.expect_overlap(
            panel,
            roof,
            axes="x",
            elem_a="sash_stile_1",
            elem_b="vent_rail_1",
            min_overlap=0.10,
            name="panel retains engagement with vent_rail_1 at full travel",
        )

    # Latch stays mounted on the panel through the slide.
    with ctx.pose({latch_joint: 0.45}):
        ctx.expect_origin_distance(
            latch,
            panel,
            axes="xyz",
            max_dist=1.6,
            name="latch remains mounted on the sliding panel when swung",
        )

    return ctx.report()


object_model = build_object_model()
