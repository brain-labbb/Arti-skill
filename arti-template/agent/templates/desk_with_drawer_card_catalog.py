from __future__ import annotations

import math
import random
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

DeskLayout = Literal["card_catalog"]
DesktopProfile = Literal["rectangular"]
SupportStyle = Literal["four_legs"]
DrawerLayout = Literal["card_catalog_grid"]
HandleStyle = Literal["bar_pull", "knob_pair", "recessed_oval", "finger_slot", "none"]
OptionalFeature = Literal["none"]
MaterialStyle = Literal["oak", "walnut", "painted"]

SOURCE_IDS = {
    "S1": (
        "data/records/rec_desk_with_drawer_196f54edeb6f4a868693c75400707c6f/"
        "revisions/rev_000001/model.py:L17-L188"
    ),
    "S2": (
        "data/records/rec_desk_with_drawer_196f54edeb6f4a868693c75400707c6f/"
        "revisions/rev_000001/model.py:L190-L250"
    ),
}

# Palettes keyed by MaterialStyle.  Each entry has:
#   "case"   - main cabinet carcass wood
#   "accent" - edge banding / dark grain accents
#   "drawer" - drawer face veneer
#   "metal"  - pull hardware (brass, nickel, etc.)
#   "paper"  - index card / label card stock
#   "label"  - cream label holder insert
PALETTES: dict[MaterialStyle, dict[str, tuple[float, float, float, float]]] = {
    "oak": {
        "case": (0.56, 0.34, 0.16, 1.0),
        "accent": (0.32, 0.19, 0.09, 1.0),
        "drawer": (0.60, 0.38, 0.18, 1.0),
        "metal": (0.82, 0.62, 0.28, 1.0),  # aged brass
        "paper": (0.93, 0.89, 0.75, 1.0),
        "label": (0.98, 0.94, 0.78, 1.0),
    },
    "walnut": {
        "case": (0.35, 0.22, 0.14, 1.0),
        "accent": (0.20, 0.12, 0.06, 1.0),
        "drawer": (0.42, 0.27, 0.17, 1.0),
        "metal": (0.71, 0.73, 0.75, 1.0),  # satin nickel
        "paper": (0.93, 0.89, 0.75, 1.0),
        "label": (0.98, 0.94, 0.78, 1.0),
    },
    "painted": {
        "case": (0.70, 0.72, 0.70, 1.0),
        "accent": (0.40, 0.41, 0.42, 1.0),
        "drawer": (0.82, 0.80, 0.74, 1.0),
        "metal": (0.22, 0.22, 0.23, 1.0),  # dark iron
        "paper": (0.93, 0.89, 0.75, 1.0),
        "label": (0.98, 0.94, 0.78, 1.0),
    },
}


@dataclass(frozen=True)
class DeskWithDrawerConfig:
    desk_layout: DeskLayout = "card_catalog"
    desktop_profile: DesktopProfile = "rectangular"
    support_style: SupportStyle = "four_legs"
    drawer_layout: DrawerLayout = "card_catalog_grid"
    handle_style: HandleStyle = "bar_pull"
    optional_feature: OptionalFeature = "none"
    material_style: MaterialStyle = "oak"
    # Overall cabinet dimensions
    cabinet_width: float = 1.20  # total width of the catalog chest
    cabinet_depth: float = 0.60  # front-to-back depth
    cabinet_height: float = 0.78  # height to top of cabinet (not including writing top)
    cabinet_bottom: float = 0.10  # height of toe-plinth / base
    top_overhang_x: float = 0.225  # total extra width of work surface beyond cabinet
    top_overhang_y: float = 0.22  # total extra depth of work surface beyond cabinet
    top_thickness: float = 0.070  # thickness of the writing top slab
    board_thickness: float = 0.035  # carcass panel thickness
    # Drawer grid
    columns: int = 4
    rows: int = 3
    divider_thickness: float = 0.025
    drawer_depth: float = 0.500  # how far the tray extends into the cabinet
    drawer_travel: float = 0.220  # how far each drawer can be pulled out
    # Label holder proportions (relative to face opening)
    label_width_frac: float = 0.72  # label frame width as fraction of opening width
    label_height_frac: float = 0.28  # label frame height as fraction of opening height
    name: str = "reference_desk_with_drawer_card_catalog"


@dataclass(frozen=True)
class ResolvedDeskWithDrawerConfig:
    desk_layout: DeskLayout
    desktop_profile: DesktopProfile
    support_style: SupportStyle
    drawer_layout: DrawerLayout
    handle_style: HandleStyle
    optional_feature: OptionalFeature
    material_style: MaterialStyle
    cabinet_width: float
    cabinet_depth: float
    cabinet_height: float
    cabinet_bottom: float
    top_overhang_x: float
    top_overhang_y: float
    top_thickness: float
    board_thickness: float
    drawer_columns: int
    drawer_rows: int
    drawer_count: int
    divider_thickness: float
    opening_width: float
    opening_height: float
    face_width: float
    face_height: float
    drawer_box_width: float
    drawer_box_height: float
    drawer_depth: float
    drawer_travel: float
    label_width_frac: float
    label_height_frac: float
    name: str
    # Derived layout: column X centres and row Z centres in cabinet-local frame
    col_centers: tuple[float, ...]
    row_centers: tuple[float, ...]
    # Y coordinate of the front face of the cabinet
    front_y: float
    back_y: float


def config_from_seed(seed: int) -> DeskWithDrawerConfig:
    rng = random.Random(seed)
    columns = rng.randint(2, 6)
    rows = rng.randint(2, 5)

    board_thickness = round(rng.uniform(0.028, 0.042), 3)
    divider_thickness = round(rng.uniform(0.018, 0.030), 3)
    cabinet_bottom = round(rng.uniform(0.08, 0.14), 3)

    # Couple cabinet footprint to drawer grid so each opening stays in a usable range.
    opening_w = rng.uniform(0.13, 0.22)
    opening_h = rng.uniform(0.10, 0.16)
    cabinet_width = round(
        2.0 * board_thickness + divider_thickness * (columns - 1) + opening_w * columns,
        3,
    )
    cabinet_height = round(
        cabinet_bottom + 2.0 * board_thickness + divider_thickness * (rows - 1) + opening_h * rows,
        3,
    )
    drawer_depth = round(rng.uniform(0.38, 0.56), 3)
    cabinet_depth = round(
        max(rng.uniform(0.48, 0.72), drawer_depth + board_thickness + 0.030),
        3,
    )
    drawer_travel = round(
        min(0.26, max(0.16, rng.uniform(0.38, 0.65) * drawer_depth)),
        3,
    )

    return DeskWithDrawerConfig(
        desk_layout="card_catalog",
        desktop_profile="rectangular",
        support_style="four_legs",
        drawer_layout="card_catalog_grid",
        handle_style=rng.choice(("bar_pull", "knob_pair", "recessed_oval", "finger_slot")),
        optional_feature="none",
        material_style=rng.choice(("oak", "walnut", "painted")),
        cabinet_width=cabinet_width,
        cabinet_depth=cabinet_depth,
        cabinet_height=cabinet_height,
        cabinet_bottom=cabinet_bottom,
        top_overhang_x=round(rng.uniform(0.12, 0.28), 3),
        top_overhang_y=round(rng.uniform(0.14, 0.26), 3),
        top_thickness=round(rng.uniform(0.050, 0.090), 3),
        board_thickness=board_thickness,
        columns=columns,
        rows=rows,
        divider_thickness=divider_thickness,
        drawer_depth=drawer_depth,
        drawer_travel=drawer_travel,
        label_width_frac=round(rng.uniform(0.60, 0.82), 3),
        label_height_frac=round(rng.uniform(0.22, 0.36), 3),
        name=f"seeded_desk_with_drawer_card_catalog_{seed}",
    )


def resolve_config(config: DeskWithDrawerConfig) -> ResolvedDeskWithDrawerConfig:
    if config.desk_layout != "card_catalog":
        raise ValueError(f"Unsupported desk_layout: {config.desk_layout}")
    if config.desktop_profile != "rectangular":
        raise ValueError(f"Unsupported desktop_profile: {config.desktop_profile}")
    if config.support_style != "four_legs":
        raise ValueError(f"Unsupported support_style: {config.support_style}")
    if config.drawer_layout != "card_catalog_grid":
        raise ValueError(f"Unsupported drawer_layout: {config.drawer_layout}")
    if config.handle_style not in {"bar_pull", "knob_pair", "recessed_oval", "finger_slot", "none"}:
        raise ValueError(f"Unsupported handle_style: {config.handle_style}")
    if config.optional_feature != "none":
        raise ValueError(f"Unsupported optional_feature: {config.optional_feature}")
    if config.material_style not in PALETTES:
        raise ValueError(f"Unsupported material_style: {config.material_style}")
    if not 2 <= config.columns <= 6:
        raise ValueError("columns must be in [2, 6]")
    if not 2 <= config.rows <= 5:
        raise ValueError("rows must be in [2, 5]")
    if not 0.12 <= config.drawer_travel <= 0.35:
        raise ValueError("drawer_travel must be in [0.12, 0.35]")

    cw = max(0.70, min(2.00, config.cabinet_width))
    cd = max(0.40, min(0.90, config.cabinet_depth))
    ch = max(0.60, min(1.00, config.cabinet_height))
    cb = max(0.06, min(0.18, config.cabinet_bottom))
    bt = max(0.022, min(0.050, config.board_thickness))
    dt = max(0.016, min(0.036, config.divider_thickness))
    cols = config.columns
    rows = config.rows

    # Keep drawer openings within practical card-catalog bounds when grid size varies.
    min_opening_w = 0.12
    min_opening_h = 0.09
    cw_min = 2.0 * bt + dt * (cols - 1) + min_opening_w * cols
    ch_min = cb + 2.0 * bt + dt * (rows - 1) + min_opening_h * rows
    cw = max(cw, cw_min)
    ch = max(ch, ch_min)

    # Compute opening sizes for the drawer grid
    inner_w = cw - 2.0 * bt
    inner_h = (ch - cb) - 2.0 * bt
    opening_w = (inner_w - dt * (cols - 1)) / cols
    opening_h = (inner_h - dt * (rows - 1)) / rows

    # Column X centers and row Z centers (in world frame, cabinet centred at x=0)
    inner_x_min = -cw / 2.0 + bt
    inner_z_min = cb + bt
    col_centers = tuple(inner_x_min + opening_w / 2.0 + c * (opening_w + dt) for c in range(cols))
    row_centers = tuple(inner_z_min + opening_h / 2.0 + r * (opening_h + dt) for r in range(rows))

    face_w = opening_w - 0.012
    face_h = opening_h - 0.014
    drawer_box_w = face_w - 0.025
    drawer_box_h = face_h - 0.026

    dd = min(config.drawer_depth, cd - bt - 0.020)
    travel = min(config.drawer_travel, dd * 0.70)

    front_y = -cd / 2.0
    back_y = cd / 2.0

    return ResolvedDeskWithDrawerConfig(
        desk_layout="card_catalog",
        desktop_profile="rectangular",
        support_style="four_legs",
        drawer_layout="card_catalog_grid",
        handle_style=config.handle_style,
        optional_feature="none",
        material_style=config.material_style,
        cabinet_width=cw,
        cabinet_depth=cd,
        cabinet_height=ch,
        cabinet_bottom=cb,
        top_overhang_x=max(0.08, min(0.32, config.top_overhang_x)),
        top_overhang_y=max(0.08, min(0.32, config.top_overhang_y)),
        top_thickness=max(0.040, min(0.100, config.top_thickness)),
        board_thickness=bt,
        drawer_columns=cols,
        drawer_rows=rows,
        drawer_count=cols * rows,
        divider_thickness=dt,
        opening_width=opening_w,
        opening_height=opening_h,
        face_width=face_w,
        face_height=face_h,
        drawer_box_width=drawer_box_w,
        drawer_box_height=drawer_box_h,
        drawer_depth=dd,
        drawer_travel=travel,
        label_width_frac=max(0.50, min(0.88, config.label_width_frac)),
        label_height_frac=max(0.18, min(0.40, config.label_height_frac)),
        name=config.name,
        col_centers=col_centers,
        row_centers=row_centers,
        front_y=front_y,
        back_y=back_y,
    )


def _add_box(part, name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _build_carcass(desk, r: ResolvedDeskWithDrawerConfig, case_mat, accent_mat) -> None:
    """Build the full open-front cabinet carcass with panels, dividers, and plinth."""
    cw, cd = r.cabinet_width, r.cabinet_depth
    ch, cb = r.cabinet_height, r.cabinet_bottom
    bt = r.board_thickness
    dt = r.divider_thickness
    front_y, back_y = r.front_y, r.back_y
    cabinet_mid_z = cb + (ch - cb) / 2.0

    # Writing work surface with edge banding
    top_w = cw + r.top_overhang_x
    top_d = cd + r.top_overhang_y
    top_z = ch + r.top_thickness / 2.0
    _add_box(desk, "work_surface", (top_w, top_d, r.top_thickness), (0.0, 0.0, top_z), case_mat)
    # Front edge band
    _add_box(
        desk,
        "front_edge_band",
        (top_w + 0.004, 0.030, r.top_thickness * 1.05),
        (0.0, front_y - r.top_overhang_y / 2.0, top_z),
        accent_mat,
    )
    # Rear edge band
    _add_box(
        desk,
        "rear_edge_band",
        (top_w + 0.004, 0.025, r.top_thickness * 0.90),
        (0.0, back_y + r.top_overhang_y / 2.0, top_z),
        accent_mat,
    )
    # Side edge bands
    for side_name, sx in (
        ("left_edge_band", -(cw / 2.0 + r.top_overhang_x / 2.0)),
        ("right_edge_band", cw / 2.0 + r.top_overhang_x / 2.0),
    ):
        _add_box(
            desk,
            side_name,
            (0.030, top_d, r.top_thickness * 1.05),
            (sx, 0.0, top_z),
            accent_mat,
        )
    # Side braces under the top overhang
    for brace_name, bx in (
        ("side_brace_left", -(cw / 2.0 - bt / 2.0 + r.top_overhang_x * 0.25)),
        ("side_brace_right", cw / 2.0 - bt / 2.0 - r.top_overhang_x * 0.25),
    ):
        _add_box(
            desk,
            brace_name,
            (bt * 1.2, cd * 0.56, r.top_thickness * 1.20),
            (bx, 0.0, ch + r.top_thickness * 0.45),
            case_mat,
        )

    # Cabinet carcass panels (open front)
    # Left and right side panels
    _add_box(
        desk,
        "side_panel_left",
        (bt, cd, ch - cb),
        (-cw / 2.0 + bt / 2.0, 0.0, cabinet_mid_z),
        case_mat,
    )
    _add_box(
        desk,
        "side_panel_right",
        (bt, cd, ch - cb),
        (cw / 2.0 - bt / 2.0, 0.0, cabinet_mid_z),
        case_mat,
    )
    # Top and bottom horizontal rails
    _add_box(
        desk,
        "top_panel",
        (cw, cd, bt),
        (0.0, 0.0, ch - bt / 2.0),
        case_mat,
    )
    _add_box(
        desk,
        "bottom_panel",
        (cw, cd, bt),
        (0.0, 0.0, cb + bt / 2.0),
        case_mat,
    )
    # Back panel
    _add_box(
        desk,
        "back_panel",
        (cw, bt, ch - cb),
        (0.0, back_y - bt / 2.0, cabinet_mid_z),
        case_mat,
    )
    # Toe plinth / base
    _add_box(
        desk,
        "toe_plinth",
        (cw * 0.92, cd * 0.86, cb),
        (0.0, 0.0, cb / 2.0),
        accent_mat,
    )

    # Internal vertical dividers (between columns)
    inner_x_min = -cw / 2.0 + bt
    inner_z_min = cb + bt
    inner_z_max = ch - bt
    inner_h = inner_z_max - inner_z_min
    inner_w = cw - 2.0 * bt
    opening_w = r.opening_width
    for c in range(1, r.drawer_columns):
        div_x = inner_x_min + c * opening_w + (c - 0.5) * dt
        _add_box(
            desk,
            f"vertical_divider_{c}",
            (dt, cd, inner_h),
            (div_x, 0.0, (inner_z_min + inner_z_max) / 2.0),
            case_mat,
        )

    # Internal horizontal dividers (between rows)
    opening_h = r.opening_height
    for row in range(1, r.drawer_rows):
        div_z = inner_z_min + row * opening_h + (row - 0.5) * dt
        _add_box(
            desk,
            f"horizontal_divider_{row}",
            (inner_w, cd, dt),
            (0.0, 0.0, div_z),
            case_mat,
        )

    # Per-opening wooden guide rails (fixed to cabinet, one pair per drawer)
    rail_w = 0.018
    rail_depth = r.drawer_depth * 0.98
    rail_center_y = front_y + 0.015 + rail_depth / 2.0
    for row_idx, zc in enumerate(r.row_centers):
        opening_bottom = zc - r.opening_height / 2.0
        # tray_bottom bottom face is at zc - drawer_box_height/2 (= drawer_bottom_z)
        drawer_bottom_z = zc - r.drawer_box_height / 2.0
        # Rail top should be at or just below the drawer bottom face so the drawer rests on rails
        rail_h = max(0.008, drawer_bottom_z - opening_bottom)
        rail_z = opening_bottom + rail_h / 2.0
        for col_idx, xc in enumerate(r.col_centers):
            for side, sx in enumerate((-1.0, 1.0)):
                rail_x = xc + sx * (r.drawer_box_width / 2.0 - 0.020)
                _add_box(
                    desk,
                    f"guide_rail_{row_idx}_{col_idx}_{side}",
                    (rail_w, rail_depth, rail_h),
                    (rail_x, rail_center_y, rail_z),
                    accent_mat,
                )


def _build_drawer(
    drawer,
    r: ResolvedDeskWithDrawerConfig,
    row_idx: int,
    col_idx: int,
    case_mat,
    metal_mat,
    paper_mat,
    label_mat,
) -> None:
    """Build a single card-catalog drawer with full box joinery, card stack, label frame, and pull."""
    face_t = 0.024
    fw = r.face_width
    fh = r.face_height
    bw = r.drawer_box_width
    bh = r.drawer_box_height
    dd = r.drawer_depth
    wall_t = 0.012
    bottom_t = 0.012

    # Front face panel (child frame origin is at the front face centre)
    _add_box(drawer, "front", (fw, face_t, fh), (0.0, 0.0, 0.0), case_mat)
    # The tray extends back from the front face along +Y
    tray_offset_y = face_t / 2.0 + dd / 2.0
    # Tray bottom
    _add_box(
        drawer,
        "tray_bottom",
        (bw, dd, bottom_t),
        (0.0, tray_offset_y, -bh / 2.0 + bottom_t / 2.0),
        case_mat,
    )
    # Side walls
    for side_name, sx in (
        ("side_wall_left", -(bw / 2.0 - wall_t / 2.0)),
        ("side_wall_right", bw / 2.0 - wall_t / 2.0),
    ):
        _add_box(
            drawer,
            side_name,
            (wall_t, dd, bh),
            (sx, tray_offset_y, 0.0),
            case_mat,
        )
    # Back wall
    _add_box(
        drawer,
        "back_wall",
        (bw, wall_t, bh),
        (0.0, face_t / 2.0 + dd - wall_t / 2.0, 0.0),
        case_mat,
    )
    # Card stack prop (simulates a full tray of index cards)
    card_stack_depth = min(dd * 0.65, 0.320)
    card_stack_h = min(bh * 0.55, 0.080)
    _add_box(
        drawer,
        "card_stack",
        (bw - 0.040, card_stack_depth, card_stack_h),
        (
            0.0,
            face_t / 2.0 + card_stack_depth / 2.0 + 0.010,
            -bh / 2.0 + bottom_t + card_stack_h / 2.0,
        ),
        paper_mat,
    )

    # Brass/metal label frame mounted through the front face
    label_frame_w = fw * r.label_width_frac
    label_frame_h = fh * r.label_height_frac
    metal_t = 0.004
    label_y = -face_t / 2.0 - metal_t / 2.0 + 0.0005
    label_z = fh * 0.14  # placed in upper-centre of face

    # Label card paper insert (set slightly inside the frame)
    _add_box(
        drawer,
        "label_card",
        (label_frame_w * 0.88, metal_t * 0.5, label_frame_h * 0.78),
        (0.0, label_y - metal_t * 0.3, label_z),
        label_mat,
    )
    # Four-piece brass frame surround
    _add_box(
        drawer,
        "label_frame_top",
        (label_frame_w + 0.010, metal_t, 0.006),
        (0.0, label_y, label_z + label_frame_h / 2.0 + 0.003),
        metal_mat,
    )
    _add_box(
        drawer,
        "label_frame_bottom",
        (label_frame_w + 0.010, metal_t, 0.006),
        (0.0, label_y, label_z - label_frame_h / 2.0 - 0.003),
        metal_mat,
    )
    _add_box(
        drawer,
        "label_frame_left",
        (0.006, metal_t, label_frame_h + 0.012),
        (-label_frame_w / 2.0 - 0.003, label_y, label_z),
        metal_mat,
    )
    _add_box(
        drawer,
        "label_frame_right",
        (0.006, metal_t, label_frame_h + 0.012),
        (label_frame_w / 2.0 + 0.003, label_y, label_z),
        metal_mat,
    )

    # Pull hardware – always uses two mounting posts + a cylindrical bar for card catalogs
    pull_z = -fh * 0.25  # lower third of face
    pull_y_out = -face_t / 2.0 - 0.026
    post_offset_x = fw * 0.28
    pull_len = fw * 0.60

    _add_box(
        drawer,
        "pull_post_left",
        (0.014, 0.026, 0.018),
        (-post_offset_x, -face_t / 2.0 - 0.013, pull_z),
        metal_mat,
    )
    _add_box(
        drawer,
        "pull_post_right",
        (0.014, 0.026, 0.018),
        (post_offset_x, -face_t / 2.0 - 0.013, pull_z),
        metal_mat,
    )
    drawer.visual(
        Cylinder(radius=0.008, length=pull_len),
        origin=Origin(xyz=(0.0, pull_y_out, pull_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="pull_bar",
    )


def build_desk_with_drawer(
    config: DeskWithDrawerConfig | None = None, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    config = config or DeskWithDrawerConfig()
    r = resolve_config(config)
    if assets is None:
        assets = AssetContext(Path(tempfile.mkdtemp(prefix="articraft-card-catalog-assets-")))
    model = ArticulatedObject(name=r.name, assets=assets)

    palette = PALETTES[r.material_style]
    case_mat = model.material("case_wood", rgba=palette["case"])
    accent_mat = model.material("accent_wood", rgba=palette["accent"])
    drawer_mat = model.material("drawer_wood", rgba=palette["drawer"])
    metal_mat = model.material("pull_hardware", rgba=palette["metal"])
    paper_mat = model.material("index_card_paper", rgba=palette["paper"])
    label_mat = model.material("label_card", rgba=palette["label"])

    # Root part: the entire cabinet carcass + work surface
    desk = model.part("desk")
    _build_carcass(desk, r, case_mat, accent_mat)
    desk.inertial = Inertial.from_geometry(
        Box((r.cabinet_width, r.cabinet_depth, r.cabinet_height + r.top_thickness)),
        mass=60.0 + r.cabinet_width * 20.0,
        origin=Origin(xyz=(0.0, 0.0, (r.cabinet_height + r.top_thickness) / 2.0)),
    )

    # One drawer part per grid cell
    for row_idx, zc in enumerate(r.row_centers):
        for col_idx, xc in enumerate(r.col_centers):
            drawer = model.part(f"drawer_{row_idx}_{col_idx}")
            _build_drawer(drawer, r, row_idx, col_idx, drawer_mat, metal_mat, paper_mat, label_mat)
            drawer.inertial = Inertial.from_geometry(
                Box((r.face_width, r.drawer_depth, r.face_height)),
                mass=1.2 + r.drawer_depth * 2.0,
                origin=Origin(xyz=(0.0, r.drawer_depth / 2.0, 0.0)),
            )

            # Slide joint: child frame origin centred on the front face of the opening.
            # The front face panel in the child frame is centred at child y=0.
            # Drawers slide outward along -Y (toward the reader).
            joint_origin = (xc, r.front_y, zc)
            limits = MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=r.drawer_travel)
            axis = (0.0, -1.0, 0.0)
            model.articulation(
                f"drawer_slide_{row_idx}_{col_idx}",
                ArticulationType.PRISMATIC,
                parent=desk,
                child=drawer,
                origin=Origin(xyz=joint_origin),
                axis=axis,
                motion_limits=limits,
            )

    return model


def build_seeded_desk_with_drawer(
    seed: int, *, assets: AssetContext | None = None
) -> ArticulatedObject:
    return build_desk_with_drawer(config_from_seed(seed), assets=assets)


def with_overrides(config: DeskWithDrawerConfig, **kwargs: object) -> DeskWithDrawerConfig:
    return replace(config, **kwargs)


def run_desk_with_drawer_tests(
    object_model: ArticulatedObject, config: DeskWithDrawerConfig
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)

    parts = {part.name for part in object_model.parts}
    joints = {joint.name for joint in object_model.articulations}

    ctx.check("desk part present", "desk" in parts)

    drawer_names = [
        f"drawer_{row}_{col}" for row in range(r.drawer_rows) for col in range(r.drawer_columns)
    ]
    slide_names = [
        f"drawer_slide_{row}_{col}"
        for row in range(r.drawer_rows)
        for col in range(r.drawer_columns)
    ]

    ctx.check(
        f"{r.drawer_count} catalog drawer parts exist",
        all(name in parts for name in drawer_names),
        details=str([n for n in drawer_names if n not in parts]),
    )
    ctx.check(
        f"{r.drawer_count} prismatic slide joints exist",
        all(name in joints for name in slide_names),
        details=str([n for n in slide_names if n not in joints]),
    )

    # All slides must be PRISMATIC and have correct axis / travel
    for sname in slide_names:
        joint = object_model.get_articulation(sname)
        ctx.check(
            f"{sname} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
        )
        ctx.check(
            f"{sname} slides along -Y (toward reader)",
            tuple(abs(v) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=str(joint.axis),
        )
        ctx.check(
            f"{sname} travel within drawer depth",
            joint.motion_limits is not None and joint.motion_limits.upper <= r.drawer_depth * 0.72,
        )

    desk = object_model.get_part("desk")

    for row_idx in range(r.drawer_rows):
        for col_idx in range(r.drawer_columns):
            drawer = object_model.get_part(f"drawer_{row_idx}_{col_idx}")
            slide = object_model.get_articulation(f"drawer_slide_{row_idx}_{col_idx}")
            guide_rail_name = f"guide_rail_{row_idx}_{col_idx}_0"

            # Check drawer rests on its guide rail (tray_bottom just above rail top)
            ctx.expect_gap(
                drawer,
                desk,
                axis="z",
                positive_elem="tray_bottom",
                negative_elem=guide_rail_name,
                max_gap=0.002,
                max_penetration=0.001,
                name=f"drawer {row_idx},{col_idx} rests on guide rail",
            )
            # Check tray_bottom is seated over (overlaps XY with) the guide rail
            ctx.expect_overlap(
                drawer,
                desk,
                axes="xy",
                elem_a="tray_bottom",
                elem_b=guide_rail_name,
                min_overlap=0.008,
                name=f"drawer {row_idx},{col_idx} is seated over guide rail",
            )

            # Check drawer slides toward the reader when extended
            rest_pos = ctx.part_world_position(drawer)
            with ctx.pose({slide: r.drawer_travel}):
                # Drawer should remain partially engaged with guide
                ctx.expect_overlap(
                    drawer,
                    desk,
                    axes="y",
                    elem_a="tray_bottom",
                    elem_b=guide_rail_name,
                    min_overlap=0.060,
                    name=f"drawer {row_idx},{col_idx} remains engaged when extended",
                )
                ext_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"drawer {row_idx},{col_idx} slides toward reader (-Y)",
                rest_pos is not None
                and ext_pos is not None
                and ext_pos[1] < rest_pos[1] - r.drawer_travel * 0.80,
                details=f"rest={rest_pos}, extended={ext_pos}",
            )

    return ctx.report()
