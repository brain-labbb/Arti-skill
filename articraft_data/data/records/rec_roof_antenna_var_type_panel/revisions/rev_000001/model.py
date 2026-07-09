from __future__ import annotations

# Rooftop flat-panel antenna on a tall vertical mast.
#
# Coordinate convention:
#   - up is +Z; the mast foot sits on the roof at z = 0.
#   - the panel's broad face normal points along the head's local +X
#     (the aiming direction).
#   - the panel is upright: tall in Z, wide in Y, thin in X.
#
# Structure / articulation:
#   - mast (root, static): tall weathered metal pole standing on the roof, with
#     two small standoff mounting brackets partway down.
#   - antenna_head (REVOLUTE about +Z, primary AZIMUTH): a rotation collar /
#     bearing hub clamped to the mast top. Aiming the antenna means swinging the
#     whole head around the vertical mast axis.
#   - panel (REVOLUTE about +Y, secondary ELEVATION tilt): the flat rectangular
#     radome panel on a stub bracket. It can be tilted up/down to trim aim.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rod(name: str, length: float, radius: float = 0.0035):
    """A straight thin rod centered at its midpoint, lying along local +X."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [(-length / 2.0, 0.0, 0.0), (0.0, 0.0, 0.0), (length / 2.0, 0.0, 0.0)],
            radius=radius,
            samples_per_segment=2,
            radial_segments=8,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_panel_antenna")

    weathered_metal = model.material("weathered_metal", rgba=(0.82, 0.81, 0.80, 1.0))
    bright_alloy = model.material("bright_alloy", rgba=(0.88, 0.89, 0.91, 1.0))
    dark_box = model.material("dark_box", rgba=(0.16, 0.17, 0.19, 1.0))
    clamp_gray = model.material("clamp_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    radome_white = model.material("radome_white", rgba=(0.90, 0.91, 0.89, 1.0))
    patch_copper = model.material("patch_copper", rgba=(0.72, 0.45, 0.20, 1.0))

    # ------------------------------------------------------------------ scales
    mast_len = 3.40          # visible mast height
    mast_r = 0.016           # ~32 mm OD steel mast
    head_z = mast_len - 0.05  # the rotation collar sits just below the mast top

    # ====================================================================== MAST
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=mast_r, length=mast_len),
        origin=Origin(xyz=(0.0, 0.0, mast_len / 2.0)),
        material=weathered_metal,
        name="mast_pole",
    )
    # small foot plate so the mast reads as planted on the roof, not floating
    mast.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=clamp_gray,
        name="foot_plate",
    )
    # two small standoff mounting brackets partway down the mast (visible tabs)
    for i, bz in enumerate((0.95, 1.95)):
        mast.visual(
            Box((0.14, 0.022, 0.014)),
            origin=Origin(xyz=(0.085, 0.0, bz)),
            material=clamp_gray,
            name=f"standoff_bracket_{i}",
        )
        # the little wall pad at the outboard end of each standoff
        mast.visual(
            Box((0.018, 0.040, 0.040)),
            origin=Origin(xyz=(0.150, 0.0, bz)),
            material=clamp_gray,
            name=f"standoff_pad_{i}",
        )
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=mast_r, length=mast_len),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, mast_len / 2.0)),
    )

    # ============================================================= ANTENNA HEAD
    # The azimuth rotation collar/bearing that clamps onto the mast top. Its part
    # frame is centered on the mast axis at head_z so azimuth spin is clean.
    antenna_head = model.part("antenna_head")
    antenna_head.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=clamp_gray,
        name="rotation_collar",
    )
    # U-bolt clamp blocks gripping the mast
    for j, zoff in enumerate((-0.035, 0.035)):
        antenna_head.visual(
            Box((0.052, 0.064, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, zoff)),
            material=dark_box,
            name=f"clamp_block_{j}",
        )
    # short stub that lifts the elevation pivot just above the collar
    antenna_head.visual(
        Box((0.040, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=clamp_gray,
        name="elevation_post",
    )
    antenna_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=0.20),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    # =========================================================== PANEL ASSEMBLY
    # The flat rectangular panel antenna rides on the elevation child.
    # The panel part origin is at the elevation pivot (top of the elevation post).
    # Panel broad face normal points along local +X (aiming direction).
    # Panel is upright: tall in Z, wide in Y, thin in X.
    panel = model.part("panel")

    # Panel dimensions (typical cellular/sector panel antenna)
    panel_w = 0.45   # width (Y)
    panel_h = 0.65   # height (Z)
    panel_d = 0.035  # depth/thickness (X)

    # The panel center sits forward of the elevation pivot and above it.
    # A stub bracket bridges from the pivot (z=0 in panel frame) to the panel.
    stub_length = 0.08   # horizontal arm from pivot to panel back
    panel_center_x = stub_length + panel_d / 2.0  # panel center X offset
    panel_center_z = panel_h / 2.0 + 0.03  # panel bottom slightly above pivot

    # --- stub bracket arm: horizontal piece from pivot to panel back ---
    # Starts at z=0 (panel frame origin = joint = post top) for physical contact.
    panel.visual(
        Box((stub_length + 0.010, 0.030, 0.028)),
        origin=Origin(xyz=(stub_length / 2.0, 0.0, 0.014)),
        material=clamp_gray,
        name="stub_bracket",
    )
    # Vertical riser connecting stub arm up to the panel back
    riser_h = panel_center_z - 0.028  # from top of stub to panel center height
    panel.visual(
        Box((0.022, 0.032, riser_h)),
        origin=Origin(xyz=(stub_length + 0.005, 0.0, 0.028 + riser_h / 2.0)),
        material=clamp_gray,
        name="panel_mount_riser",
    )

    # --- main radome panel (CadQuery for rounded edges) ---
    # Build centered at origin in CadQuery, then place via visual origin.
    panel_shape = (
        cq.Workplane("XY")
        .box(panel_d, panel_w, panel_h)
        .edges("|Z").fillet(0.012)
    )
    panel.visual(
        mesh_from_cadquery(panel_shape, "radome_panel"),
        origin=Origin(xyz=(panel_center_x, 0.0, panel_center_z)),
        material=radome_white,
        name="radome_panel",
    )

    # --- patch element grid on the front face (+X side) ---
    # A regular grid of small shallow raised rectangular patches, typical of
    # a patch array antenna behind a radome.
    n_cols = 4   # columns along Y
    n_rows = 6   # rows along Z
    patch_w = 0.065  # patch width (Y)
    patch_h = 0.055  # patch height (Z)
    patch_depth = 0.004  # how far patches protrude from the face

    # Front face of panel in part-local coords
    front_x = panel_center_x + panel_d / 2.0

    # Spacing: evenly distributed within the panel area
    y_spacing = panel_w / (n_cols + 1)
    z_spacing = panel_h / (n_rows + 1)

    for i in range(n_rows):
        for j in range(n_cols):
            idx = i * n_cols + j
            py = -panel_w / 2.0 + y_spacing * (j + 1)
            pz = panel_center_z - panel_h / 2.0 + z_spacing * (i + 1)
            # Patches sit proud of the front face (slight embed for connectivity)
            px = front_x + patch_depth / 2.0 - 0.001

            panel.visual(
                Box((patch_depth, patch_w, patch_h)),
                origin=Origin(xyz=(px, py, pz)),
                material=patch_copper,
                name=f"patch_{idx}",
            )

    # --- small junction box on the panel back ---
    panel.visual(
        Box((0.055, 0.045, 0.040)),
        origin=Origin(xyz=(panel_center_x - panel_d / 2.0 - 0.027, 0.0, panel_center_z - 0.10)),
        material=dark_box,
        name="junction_box",
    )

    panel.inertial = Inertial.from_geometry(
        Box((panel_d + 0.10, panel_w, panel_h)),
        mass=3.5,
        origin=Origin(xyz=(panel_center_x, 0.0, panel_center_z)),
    )

    # ============================================================ ARTICULATIONS
    # PRIMARY: azimuth rotation of the whole head about the vertical mast axis.
    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=antenna_head,
        origin=Origin(xyz=(0.0, 0.0, head_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    # SECONDARY: elevation tilt of the panel about the horizontal -Y axis.
    # Panel extends along local +X from the pivot; -Y axis lifts the front (+X)
    # end upward for positive q, so the antenna can be aimed slightly up/down.
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=antenna_head,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    head = object_model.get_part("antenna_head")
    panel = object_model.get_part("panel")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_tilt")

    # --- intentional joint-fit overlaps --------------------------------------
    ctx.allow_overlap(
        head,
        mast,
        elem_a="rotation_collar",
        elem_b="mast_pole",
        reason="The azimuth rotation collar clamps around the mast top; the bearing fit is intentional.",
    )
    for cb in ("clamp_block_0", "clamp_block_1"):
        ctx.allow_overlap(
            head,
            mast,
            elem_a=cb,
            elem_b="mast_pole",
            reason="U-bolt clamp blocks grip around the mast pole to hold the rotor collar.",
        )
    ctx.allow_overlap(
        head,
        panel,
        elem_a="elevation_post",
        elem_b="stub_bracket",
        reason="The elevation pivot post meets the stub bracket at the tilt joint.",
    )

    # --- base sits on the roof at z ~ 0 (not buried, not floating) ------------
    mast_aabb = ctx.part_world_aabb(mast)
    ctx.check(
        "mast foot rests on the roof at z~0",
        mast_aabb is not None and abs(mast_aabb[0][2]) < 0.02,
        details=f"mast world aabb min z = {None if mast_aabb is None else mast_aabb[0][2]}",
    )

    # --- mast is a tall vertical pole ----------------------------------------
    ctx.check(
        "mast is tall (>3 m)",
        mast_aabb is not None and (mast_aabb[1][2] - mast_aabb[0][2]) > 3.0,
        details=f"mast height = {None if mast_aabb is None else mast_aabb[1][2] - mast_aabb[0][2]}",
    )

    # --- the head/panel assembly sits at the mast top -------------------------
    head_pos = ctx.part_world_position(head)
    ctx.check(
        "antenna head mounted near the mast top",
        head_pos is not None and head_pos[2] > 3.0,
        details=f"head world z = {None if head_pos is None else head_pos[2]}",
    )

    # --- PRIMARY: azimuth is a vertical-axis revolute joint ------------------
    ctx.check(
        "azimuth joint is revolute about +Z",
        azimuth.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    # spinning azimuth should swing the panel around in X/Y
    panel_rest = _aabb_center(ctx.part_element_world_aabb(panel, elem="radome_panel"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        panel_spun = _aabb_center(ctx.part_element_world_aabb(panel, elem="radome_panel"))
    ctx.check(
        "azimuth rotation swings the panel antenna horizontally",
        abs(panel_spun[0] - panel_rest[0]) > 0.05 or abs(panel_spun[1] - panel_rest[1]) > 0.05,
        details=f"rest={panel_rest}, spun={panel_spun}",
    )

    # --- SECONDARY: elevation is a horizontal-axis revolute joint ------------
    ctx.check(
        "elevation joint is revolute about the horizontal Y axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    # tilting elevation up should raise the front face center of the panel
    # (a patch element on the front face, near the middle of the panel)
    front_rest = _aabb_center(ctx.part_element_world_aabb(panel, elem="patch_10"))
    with ctx.pose({elevation: 0.35}):
        front_tilted = _aabb_center(ctx.part_element_world_aabb(panel, elem="patch_10"))
    ctx.check(
        "elevation tilt raises the front of the panel (aiming upward)",
        front_tilted[2] > front_rest[2] + 0.005,
        details=f"rest front z={front_rest[2]}, tilted front z={front_tilted[2]}",
    )

    # --- panel is a flat rectangular shape (wide and tall, thin in X) --------
    panel_aabb = ctx.part_element_world_aabb(panel, elem="radome_panel")
    if panel_aabb is not None:
        panel_dx = panel_aabb[1][0] - panel_aabb[0][0]
        panel_dy = panel_aabb[1][1] - panel_aabb[0][1]
        panel_dz = panel_aabb[1][2] - panel_aabb[0][2]
    else:
        panel_dx = panel_dy = panel_dz = 0.0

    ctx.check(
        "radome panel is a flat rectangle (tall and wide, thin in depth)",
        panel_dy > 0.30 and panel_dz > 0.40 and panel_dx < 0.08,
        details=f"panel dims: dx={panel_dx:.4f}, dy={panel_dy:.4f}, dz={panel_dz:.4f}",
    )

    # --- patch grid exists on the panel front face ---
    patch_first = ctx.part_element_world_aabb(panel, elem="patch_0")
    patch_last = ctx.part_element_world_aabb(panel, elem="patch_23")
    ctx.check(
        "patch element grid is present on the panel face",
        patch_first is not None and patch_last is not None,
        details="patch_0 and patch_23 must exist",
    )

    # --- patches are in front of the panel (larger X than panel center) -------
    if patch_first is not None and panel_aabb is not None:
        panel_center_x = (panel_aabb[0][0] + panel_aabb[1][0]) / 2.0
        patch_center_x = (patch_first[0][0] + patch_first[1][0]) / 2.0
        ctx.check(
            "patches are on the front (+X aiming) face of the panel",
            patch_center_x > panel_center_x,
            details=f"panel center x={panel_center_x:.4f}, patch x={patch_center_x:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
