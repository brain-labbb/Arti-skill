"""Foosball / table-football table modular template.

Structure (pattern = ``mixed``): a fixed rigid ``cabinet`` parent (black side
panels with rounded corners + rod-clearance bores, green pitch with raised white
markings, end walls with open goal slots, gray top rim, bead score rails) plus
two independent multiplicity axes and two fixed named slots:

  * Slot A ``rod_count`` (PRIMARY multiplicity, even N in [2,8]): N player-rod
    assemblies evenly spaced along X. Each rod keeps the IMMUTABLE two-DOF spine
    — a PRISMATIC ``rod_{i}_slide`` (massless carrier, axis Y) + a CONTINUOUS
    ``rod_{i}_spin`` (axis Y). This spine survives every leg/grip/figure swap.
  * Slot B ``leg_form`` (4): splayed_legs_4 / side_panels_full_height /
    cross_x_trestles / tubular_legs_4 — under-cabinet support to the floor,
    inlined as non-moving visuals on ``cabinet`` (Rule 1).
  * Slot C ``grip_form`` (3, per-rod): plain_cylinder_grip /
    contoured_waisted_grip / ball_knob_grip — single ``handle`` visual on the
    team-side rod end, fully outside the cabinet wall.
  * Slot D ``figures_policy`` (2) + nested ``figures_per_rod`` (per-rod n in
    [1,5]): mixed_figures (positional GK1/D2/M5/A3) vs uniform_figures (constant
    N every rod). Figures are fixed visuals on ``rod_{i}`` (ride slide + spin).

Sourced from the reviewed spec ``specs_modular_v1/Sports_Table_football.md`` and the
9 five-star samples (parent + 8 slot-fork variants) under ``data/records/``.

Rule notes: figures / grips / score rails are non-moving ``part.visual`` (Rule
1). Every rod has its two non-FIXED joints; the rod shaft runs through the
side-panel bearing bores under element-scoped ``allow_overlap`` (captured-rod
geometry, MatingContract grandfathered like the source record).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Literal

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

__modular__ = True

LegForm = Literal[
    "splayed_legs_4",
    "side_panels_full_height",
    "cross_x_trestles",
    "tubular_legs_4",
]
GripForm = Literal[
    "plain_cylinder_grip",
    "contoured_waisted_grip",
    "ball_knob_grip",
]
FiguresPolicy = Literal["mixed_figures", "uniform_figures"]
PaletteStyle = Literal[
    "classic_black_red_blue",
    "arcade_blue_yellow_white",
    "natural_wood_red_blue",
    "pro_charcoal_silver",
    "retro_green_orange",
]

LEG_FORMS: tuple[LegForm, ...] = (
    "splayed_legs_4",
    "side_panels_full_height",
    "cross_x_trestles",
    "tubular_legs_4",
)
GRIP_FORMS: tuple[GripForm, ...] = (
    "plain_cylinder_grip",
    "contoured_waisted_grip",
    "ball_knob_grip",
)
FIGURES_POLICIES: tuple[FiguresPolicy, ...] = ("mixed_figures", "uniform_figures")
PALETTE_STYLES: tuple[PaletteStyle, ...] = (
    "classic_black_red_blue",
    "arcade_blue_yellow_white",
    "natural_wood_red_blue",
    "pro_charcoal_silver",
    "retro_green_orange",
)

# rod_count: even-only (per-team symmetry), weighted toward canonical 6/8.
ROD_COUNTS: tuple[int, ...] = (4, 6, 8)
ROD_COUNT_WEIGHTS = (0.22, 0.40, 0.38)
UNIFORM_FIG_CHOICES: tuple[int, ...] = (2, 3)

# ---------------------------------------------------------------- palettes
# keys: cabinet / rim / pitch / mark / leg / chrome / steel / team_a / team_b
PALETTES: dict[PaletteStyle, dict[str, tuple[float, float, float, float]]] = {
    "classic_black_red_blue": {
        "cabinet": (0.05, 0.05, 0.06, 1.0),
        "rim": (0.62, 0.63, 0.65, 1.0),
        "pitch": (0.10, 0.45, 0.16, 1.0),
        "mark": (0.95, 0.95, 0.95, 1.0),
        "leg": (0.92, 0.92, 0.93, 1.0),
        "chrome": (0.80, 0.81, 0.83, 1.0),
        "steel": (0.68, 0.70, 0.72, 1.0),
        "team_a": (0.78, 0.10, 0.08, 1.0),
        "team_b": (0.10, 0.22, 0.68, 1.0),
    },
    "arcade_blue_yellow_white": {
        "cabinet": (0.07, 0.12, 0.30, 1.0),
        "rim": (0.93, 0.94, 0.95, 1.0),
        "pitch": (0.10, 0.42, 0.18, 1.0),
        "mark": (0.97, 0.97, 0.97, 1.0),
        "leg": (0.90, 0.91, 0.93, 1.0),
        "chrome": (0.82, 0.83, 0.85, 1.0),
        "steel": (0.70, 0.72, 0.74, 1.0),
        "team_a": (0.94, 0.78, 0.10, 1.0),
        "team_b": (0.95, 0.95, 0.96, 1.0),
    },
    "natural_wood_red_blue": {
        "cabinet": (0.55, 0.36, 0.18, 1.0),
        "rim": (0.72, 0.55, 0.34, 1.0),
        "pitch": (0.12, 0.46, 0.18, 1.0),
        "mark": (0.96, 0.95, 0.92, 1.0),
        "leg": (0.50, 0.32, 0.16, 1.0),
        "chrome": (0.66, 0.50, 0.30, 1.0),
        "steel": (0.70, 0.71, 0.72, 1.0),
        "team_a": (0.80, 0.12, 0.10, 1.0),
        "team_b": (0.12, 0.24, 0.66, 1.0),
    },
    "pro_charcoal_silver": {
        "cabinet": (0.13, 0.14, 0.15, 1.0),
        "rim": (0.78, 0.79, 0.81, 1.0),
        "pitch": (0.08, 0.40, 0.15, 1.0),
        "mark": (0.96, 0.96, 0.96, 1.0),
        "leg": (0.80, 0.81, 0.83, 1.0),
        "chrome": (0.85, 0.86, 0.88, 1.0),
        "steel": (0.74, 0.76, 0.78, 1.0),
        "team_a": (0.80, 0.10, 0.10, 1.0),
        "team_b": (0.12, 0.24, 0.70, 1.0),
    },
    "retro_green_orange": {
        "cabinet": (0.18, 0.34, 0.22, 1.0),
        "rim": (0.90, 0.86, 0.74, 1.0),
        "pitch": (0.14, 0.44, 0.20, 1.0),
        "mark": (0.95, 0.93, 0.86, 1.0),
        "leg": (0.52, 0.36, 0.20, 1.0),
        "chrome": (0.66, 0.50, 0.32, 1.0),
        "steel": (0.68, 0.70, 0.70, 1.0),
        "team_a": (0.88, 0.45, 0.10, 1.0),
        "team_b": (0.18, 0.52, 0.24, 1.0),
    },
}

# ---------------------------------------------------------------- dimensions
CAB_L = 1.20  # X, table length (nominal; scaled by cabinet_length_scale)
CAB_W = 0.70  # Y, table width
WALL_T = 0.025
BASE_BOT = 0.60  # cabinet underside height
BASE_TOP = 0.616  # black base panel top
PITCH_TOP = 0.622  # green pitch playing surface
WALL_TOP = 0.90
RIM_TOP = 0.914

SIDE_Y = CAB_W / 2.0 - WALL_T / 2.0  # 0.3375 side panel center
INNER_Y = CAB_W / 2.0 - WALL_T  # 0.325 interior half width

GOAL_HALF_W = 0.10
GOAL_TOP = PITCH_TOP + 0.103  # lintel underside

ROD_R = 0.008
ROD_Z = PITCH_TOP + 0.082  # 0.704 rod axis height
HOLE_R = 0.0072  # bearing bore (interference fit)

MARK_T = 0.0012
MARK_ZC = PITCH_TOP + MARK_T / 2.0

# splayed legs
LEG_TILT = 0.13
LEG_LEN = 0.6116
LEG_SX, LEG_SY = 0.06, 0.095

# tubular legs
LEG_TUBE_R = 0.025
LEG_FOOT_R = 0.035
LEG_FOOT_H = 0.012
LEG_TUBE_LEN = BASE_BOT - LEG_FOOT_H
LEG_INSET_X = 0.06
LEG_INSET_Y = 0.06

# cross trestle
TRESTLE_TOP_SPREAD = 0.24
TRESTLE_FOOT_SPREAD = 0.26
TRESTLE_BAR_T = 0.040
TRESTLE_X = 0.50  # |x| of each trestle (clears goal mouth at END_X)

# grip geometry
GRIP_STEM_LEN = 0.040
GRIP_STEM_R = 0.013
GRIP_BALL_R = 0.024
PLAIN_GRIP_R = 0.0175
PLAIN_GRIP_LEN = 0.115
CONTOUR_HALF_L = 0.0575

# figure layout (local to rod frame, hangs along -Z at q=0)
FIG_TORSO = (0.024, 0.032, 0.052)
FIG_TORSO_Z = -0.010
FIG_HEAD_R = 0.0125
FIG_HEAD_Z = 0.0265
FIG_LEG = (0.017, 0.020, 0.028)
FIG_LEG_Z = -0.048
FIG_FOOT = (0.018, 0.022, 0.012)
FIG_FOOT_Z = -0.0625
FIG_FOOT_X = 0.003
FIG_FOOT_TILT = 0.25

PROT_MARGIN = 0.06  # rod protrudes this far beyond the wall at extreme slide
ROD_SPAN_FRAC = 0.875  # DESIRED rod span fraction of CAB_L (pulled in if the
# outermost figures' spin envelope would otherwise reach the end walls)
END_WALL_INSET = 0.03  # end wall sits this far inside the cabinet edge (X)
# The outermost rod is placed so its full figure spin envelope (radius swing_r)
# stays at least END_CLEAR short of the end-wall posts/lintel in X. This makes
# an end player sweeping into a wall post geometrically impossible at ANY
# slide+spin, since spin moves figures in X and slide only in Y.
END_CLEAR = 0.010
# Adjacent spinning rods keep at least this gap between their figure envelopes.
SPIN_CLEARANCE = 0.010


# ============================================================ small helpers
def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pick(value, choices):
    return value if value in choices else choices[0]


def rod_protrusion(travel: float) -> float:
    return travel + PROT_MARGIN


def figure_swing_radius(scale: float = 1.0) -> float:
    """Max radial distance of any figure point from the rod axis (XZ plane).

    ``scale`` uniformly shrinks the figure geometry AND its mounting offsets about
    the rod axis, so the swing radius scales linearly with ``scale``.
    """
    c, s = math.cos(FIG_FOOT_TILT), math.sin(FIG_FOOT_TILT)
    hx, hz = scale * FIG_FOOT[0] / 2.0, scale * FIG_FOOT[2] / 2.0
    r_max = 0.0
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            x = scale * FIG_FOOT_X + sx * hx * c + sz * hz * s
            z = scale * FIG_FOOT_Z - sx * hx * s + sz * hz * c
            r_max = max(r_max, math.hypot(x, z))
    r_max = max(r_max, scale * (abs(FIG_HEAD_Z) + FIG_HEAD_R))
    r_max = max(
        r_max,
        scale * math.hypot(abs(FIG_TORSO_Z) + FIG_TORSO[2] / 2.0, FIG_TORSO[0] / 2.0),
    )
    r_max = max(
        r_max,
        scale * math.hypot(abs(FIG_LEG_Z) + FIG_LEG[2] / 2.0, FIG_LEG[0] / 2.0),
    )
    return r_max


# ============================================================ rod layout
def _team_for_index(idx: int, n_rods: int) -> str:
    """Alternate-ish team assignment, symmetric: outer goalies, then mix.

    We mirror the parent's red/blue interleave: the -X half leans red, +X half
    leans blue, with goalies at the extreme ends.
    """
    # left half -> "red", right half -> "blue", but interleave inner rods so the
    # field reads like real foosball (defenders/attackers face off).
    half = n_rods // 2
    if idx < half:
        # near -X end: alternate red/blue but goalie (idx 0) is red
        return "red" if (idx % 2 == 0) else "blue"
    # near +X end: mirror
    j = idx - half
    return "blue" if (j % 2 == (half - 1) % 2) else "red"


def _figure_count_mixed(idx: int, n_rods: int) -> tuple[int, float]:
    """Positional GK/D/M/A figure count + spacing for the mixed policy.

    Role pattern per half mirrors real foosball. Returns (n_fig, spacing).
    """
    half = n_rods // 2
    pos = idx if idx < half else (n_rods - 1 - idx)  # distance from nearest end
    # pos 0 = goalie, 1 = defense, 2 = midfield, 3 = attack (clamped)
    if half <= 1:
        roles = [(1, 0.0)]
    elif half == 2:
        roles = [(1, 0.0), (3, 0.185)]  # GK + combined outfield (mini table)
    elif half == 3:
        roles = [(1, 0.0), (2, 0.24), (3, 0.185)]  # GK / D / A
    else:  # half >= 4 (n_rods=8): GK / D / M / A
        roles = [(1, 0.0), (2, 0.24), (5, 0.12), (3, 0.185)]
    pos = min(pos, len(roles) - 1)
    return roles[pos]


def _travel_for(n_fig: int, rod_spacing: float, travel_scale: float) -> float:
    """Per-rod slide travel: denser figures -> shorter travel."""
    base = {1: 0.115, 2: 0.175, 3: 0.110, 5: 0.055}.get(n_fig, 0.110)
    travel = base * travel_scale
    # never let travel push figures past the interior walls (checked again later)
    return travel


@dataclass(frozen=True)
class RodSpec:
    idx: int  # 1-based
    x: float
    team: str  # "red" / "blue"
    n_fig: int
    spacing: float
    travel: float


@dataclass(frozen=True)
class FoosballConfig:
    leg_form: LegForm | None = None
    grip_form: GripForm | None = None
    figures_policy: FiguresPolicy | None = None
    rod_count: int | None = None
    uniform_fig: int | None = None  # constant n_fig when figures_policy=uniform
    palette_style: PaletteStyle = "classic_black_red_blue"
    cabinet_length_scale: float = 1.0
    leg_splay_scale: float = 1.0
    grip_radius_scale: float = 1.0
    slide_travel_scale: float = 1.0
    name: str = "foosball_table"


@dataclass(frozen=True)
class ResolvedConfig:
    leg_form: LegForm
    grip_form: GripForm
    figures_policy: FiguresPolicy
    rod_count: int
    palette_style: PaletteStyle
    cab_l: float
    rod_spacing: float
    grip_radius_scale: float
    leg_splay_scale: float
    rods: tuple[RodSpec, ...]
    swing_r: float
    fig_scale: float
    end_in_x: float
    name: str


def config_from_seed(seed: int) -> FoosballConfig:
    rng = random.Random(seed)
    return FoosballConfig(
        leg_form=rng.choice(LEG_FORMS),
        grip_form=rng.choice(GRIP_FORMS),
        figures_policy=rng.choice(FIGURES_POLICIES),
        rod_count=rng.choices(ROD_COUNTS, weights=ROD_COUNT_WEIGHTS, k=1)[0],
        uniform_fig=rng.choice(UNIFORM_FIG_CHOICES),
        palette_style=rng.choice(PALETTE_STYLES),
        cabinet_length_scale=round(rng.uniform(0.92, 1.10), 4),
        leg_splay_scale=round(rng.uniform(0.70, 1.30), 4),
        grip_radius_scale=round(rng.uniform(0.85, 1.15), 4),
        slide_travel_scale=round(rng.uniform(0.85, 1.10), 4),
        name=f"seeded_foosball_table_{seed}",
    )


def resolve_config(config: FoosballConfig | None = None) -> ResolvedConfig:
    cfg = config or FoosballConfig()
    palette_style = _pick(cfg.palette_style, PALETTE_STYLES)
    leg_form = _pick(cfg.leg_form, LEG_FORMS)
    grip_form = _pick(cfg.grip_form, GRIP_FORMS)
    figures_policy = _pick(cfg.figures_policy, FIGURES_POLICIES)

    rod_count = int(cfg.rod_count) if cfg.rod_count is not None else 8
    rod_count = int(_clamp(rod_count, 2, 8))
    if rod_count % 2 != 0:  # even-only (per-team symmetry)
        rod_count -= 1
    rod_count = max(2, rod_count)

    len_scale = _clamp(cfg.cabinet_length_scale, 0.92, 1.10)
    splay_scale = _clamp(cfg.leg_splay_scale, 0.70, 1.30)
    grip_r_scale = _clamp(cfg.grip_radius_scale, 0.85, 1.15)
    travel_scale = _clamp(cfg.slide_travel_scale, 0.85, 1.10)

    cab_l = CAB_L * len_scale

    # --- co-solve rod span + figure scale against BOTH wall constraints ---
    # (a) neighbour: adjacent spinning rods' figure envelopes must clear:
    #         2*swing_r + SPIN_CLEARANCE <= rod_spacing
    # (b) end wall: the OUTERMOST rod's figure spin envelope must stay short of
    #     the end-wall post near face in X:
    #         outer_x + swing_r + END_CLEAR <= END_IN_X
    # Both constraints relax as the figures shrink, so a short fixed-point over
    # (outer_x, fig_scale) converges deterministically. The desired span is
    # ROD_SPAN_FRAC; the outer rods are pulled in only as far as (b) requires.
    swing_base = figure_swing_radius(1.0)
    end_in_x = cab_l / 2.0 - END_WALL_INSET - WALL_T / 2.0  # post near face (X)
    outer_x_desired = 0.5 * ROD_SPAN_FRAC * cab_l

    fig_scale = 1.0
    swing_r = swing_base
    outer_x = outer_x_desired
    rod_spacing = 0.0
    for _ in range(50):
        # (b) cap the outer rod so the spin envelope clears the end wall.
        outer_cap = end_in_x - swing_r - END_CLEAR
        outer_x = _clamp(outer_x_desired, 0.02, max(0.02, outer_cap))
        rod_spacing = (2.0 * outer_x / (rod_count - 1)) if rod_count >= 2 else 2.0 * outer_x
        # (a) shrink figures until adjacent spin envelopes clear.
        new_scale = 1.0
        if 2.0 * swing_base > 0.0:
            new_scale = min(1.0, (rod_spacing - SPIN_CLEARANCE) / (2.0 * swing_base))
        new_scale = _clamp(new_scale, 0.55, 1.0)
        new_swing = figure_swing_radius(new_scale)
        if abs(new_swing - swing_r) < 1e-7 and abs(new_scale - fig_scale) < 1e-7:
            fig_scale, swing_r = new_scale, new_swing
            break
        fig_scale, swing_r = new_scale, new_swing

    # Recompute the span from the CONVERGED swing_r so the geometry and the
    # end-wall invariant (outer_x + swing_r + END_CLEAR <= end_in_x) hold
    # exactly, not just to the fixed-point tolerance.
    outer_cap = end_in_x - swing_r - END_CLEAR
    outer_x = _clamp(outer_x_desired, 0.02, max(0.02, outer_cap))
    rod_spacing = (2.0 * outer_x / (rod_count - 1)) if rod_count >= 2 else 2.0 * outer_x

    # grip radius must not exceed half the rod spacing (avoid neighbor clash).
    max_grip_r = max(0.010, 0.5 * rod_spacing - 0.004)
    grip_r_scale = min(grip_r_scale, max_grip_r / PLAIN_GRIP_R)

    uniform_fig = int(cfg.uniform_fig) if cfg.uniform_fig is not None else 3
    uniform_fig = int(_clamp(uniform_fig, 1, 5))

    rods: list[RodSpec] = []
    x0 = -0.5 * (rod_count - 1) * rod_spacing
    for i in range(rod_count):
        x = x0 + i * rod_spacing
        team = _team_for_index(i, rod_count)
        if figures_policy == "uniform_figures":
            n_fig = uniform_fig
            spacing = 0.0 if n_fig == 1 else 0.185
        else:
            n_fig, spacing = _figure_count_mixed(i, rod_count)

        # --- figure-clearance + interior-wall projection (inequalities) ---
        # 1) the n_fig row, centered on the rod, must stay inside the interior
        #    walls even at the slide extreme. half_row + travel <= INNER_Y - eps.
        travel = _travel_for(n_fig, rod_spacing, travel_scale)
        if n_fig >= 2:
            half_row = (n_fig - 1) / 2.0 * spacing
            # shrink spacing first, then travel, to satisfy containment.
            for _ in range(60):
                half_row = (n_fig - 1) / 2.0 * spacing
                if half_row + travel + FIG_TORSO[1] / 2.0 <= INNER_Y - 0.004:
                    break
                if spacing > 0.10:
                    spacing = max(0.10, spacing - 0.006)
                elif travel > 0.04:
                    travel = max(0.04, travel - 0.006)
                else:
                    break
        else:
            # goalie: ensure travel keeps the single figure inside the walls.
            for _ in range(20):
                if travel + FIG_TORSO[1] / 2.0 <= INNER_Y - 0.004:
                    break
                travel = max(0.04, travel - 0.006)

        # 2) neighbor spin clearance: 2*swing_r < rod_spacing (global; checked
        #    via spacing). Already guaranteed by rod_spacing >> 2*swing_r for
        #    N<=8 over our span, but clamp defensively in tests.
        rods.append(RodSpec(idx=i + 1, x=x, team=team, n_fig=n_fig, spacing=spacing, travel=travel))

    return ResolvedConfig(
        leg_form=leg_form,
        grip_form=grip_form,
        figures_policy=figures_policy,
        rod_count=rod_count,
        palette_style=palette_style,
        cab_l=cab_l,
        rod_spacing=rod_spacing,
        grip_radius_scale=grip_r_scale,
        leg_splay_scale=splay_scale,
        rods=tuple(rods),
        swing_r=swing_r,
        fig_scale=fig_scale,
        end_in_x=end_in_x,
        name=cfg.name,
    )


def slot_choices_for_config(r: ResolvedConfig) -> tuple[tuple[str, str], ...]:
    # nested figure signature: distinct per (policy, rod_count, figure-count vector)
    fig_vec = "-".join(str(rod.n_fig) for rod in r.rods)
    return (
        ("leg_form", r.leg_form),
        ("grip_form", r.grip_form),
        ("figures_policy", r.figures_policy),
        ("rod_count", f"n{r.rod_count}"),
        ("figures_per_rod", fig_vec),
    )


def slot_choices_for_seed(seed: int) -> tuple[tuple[str, str], ...]:
    return slot_choices_for_config(resolve_config(config_from_seed(seed)))


# ============================================================ cabinet meshes
def _side_panel_mesh(name: str, r: ResolvedConfig, *, full_height: bool, assets):
    """Black side panel with rounded corners + rod clearance bores.

    full_height=False: panel spans BASE_TOP..WALL_TOP (legs carry the floor).
    full_height=True: panel drops to the floor (side_panels_full_height leg).
    Authored centered in X/Z; solid spans local y in [-WALL_T, 0].
    """
    if full_height:
        height = WALL_TOP
        zc = WALL_TOP / 2.0
        fillet = 0.035
    else:
        height = WALL_TOP - BASE_TOP
        zc = (WALL_TOP + BASE_TOP) / 2.0
        fillet = 0.045
    panel = cq.Workplane("XZ").rect(r.cab_l, height).extrude(WALL_T).edges("|Y").fillet(fillet)
    pts = [(rod.x, ROD_Z - zc) for rod in r.rods]
    holes = cq.Workplane("XZ").pushPoints(pts).circle(HOLE_R).extrude(WALL_T * 3.0, both=True)
    return mesh_from_cadquery(panel.cut(holes), name, assets=assets)


def _tubular_leg_mesh(name: str, *, assets):
    tube = cq.Workplane("XY").circle(LEG_TUBE_R).extrude(-LEG_TUBE_LEN)
    foot = (
        cq.Workplane("XY").workplane(offset=-LEG_TUBE_LEN).circle(LEG_FOOT_R).extrude(-LEG_FOOT_H)
    )
    return mesh_from_cadquery(tube.union(foot), name, assets=assets)


def _ergonomic_grip_mesh(name: str, scale: float, *, assets):
    half_L = CONTOUR_HALF_L
    profile = [
        (0.0, -half_L),
        (0.015 * scale, -half_L + 0.006),
        (0.019 * scale, -half_L + 0.020),
        (0.017 * scale, -half_L + 0.032),
        (0.0125 * scale, 0.0),
        (0.017 * scale, half_L - 0.032),
        (0.019 * scale, half_L - 0.020),
        (0.015 * scale, half_L - 0.006),
        (0.0, half_L),
    ]
    wp = cq.Workplane("XZ")
    wp = wp.moveTo(profile[0][0], profile[0][1]).spline(profile[1:], includeCurrent=True).close()
    grip = wp.revolve(360, (0, 0), (0, 1))
    return mesh_from_cadquery(grip, name, assets=assets)


def _knob_grip_mesh(name: str, scale: float, *, assets):
    stem = cq.Workplane("XZ").circle(GRIP_STEM_R * scale).extrude(GRIP_STEM_LEN)
    ball = cq.Workplane("XZ").workplane(offset=GRIP_STEM_LEN).sphere(GRIP_BALL_R * scale)
    return mesh_from_cadquery(stem.union(ball), name, assets=assets)


# ============================================================ leg builders
def _emit_splayed_legs(cab, r: ResolvedConfig, mats) -> None:
    tilt = LEG_TILT * r.leg_splay_scale
    half = LEG_LEN / 2.0
    leg_x = r.cab_l / 2.0 - 0.10  # inset from cabinet end
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        cab.visual(
            Box((LEG_SX, LEG_SY, LEG_LEN)),
            origin=Origin(
                xyz=(
                    sx * (leg_x + half * math.sin(tilt)),
                    sy * 0.27,
                    0.610 - half * math.cos(tilt),
                ),
                rpy=(0.0, -sx * tilt, 0.0),
            ),
            material=mats["leg"],
            name=f"leg_{i}",
        )


def _emit_tubular_legs(cab, r: ResolvedConfig, mats, *, assets) -> None:
    leg_x = r.cab_l / 2.0 - LEG_INSET_X
    leg_y = CAB_W / 2.0 - LEG_INSET_Y
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        cab.visual(
            _tubular_leg_mesh(f"leg_mesh_{i}", assets=assets),
            origin=Origin(xyz=(sx * leg_x, sy * leg_y, BASE_BOT)),
            material=mats["chrome"],
            name=f"leg_{i}",
        )


def _emit_cross_trestles(cab, r: ResolvedConfig, mats) -> None:
    height = BASE_BOT
    bar_t = TRESTLE_BAR_T
    top_s = TRESTLE_TOP_SPREAD * r.leg_splay_scale
    foot_s = TRESTLE_FOOT_SPREAD * r.leg_splay_scale
    dy = top_s + foot_s
    bar_len = math.hypot(dy, height)
    bar_tilt = math.atan2(dy, height)
    for idx, x_off in enumerate((-TRESTLE_X, TRESTLE_X)):
        for sign, letter in ((-1, "a"), (1, "b")):
            y_center = -sign * (foot_s - top_s) / 2.0
            roll = -sign * bar_tilt
            cab.visual(
                Box((bar_t, bar_t, bar_len)),
                origin=Origin(xyz=(x_off, y_center, height / 2.0), rpy=(roll, 0.0, 0.0)),
                material=mats["leg"],
                name=f"trestle_{idx}_bar_{letter}",
            )
        brace_y = 2.0 * foot_s * 0.70
        cab.visual(
            Box((bar_t, brace_y, bar_t)),
            origin=Origin(xyz=(x_off, 0.0, height * 0.38)),
            material=mats["leg"],
            name=f"trestle_{idx}_brace",
        )


# ============================================================ grip builder
def _emit_handle(rod, r: ResolvedConfig, mats, *, side: float, end_y: float, team_mat, assets):
    scale = r.grip_radius_scale
    if r.grip_form == "plain_cylinder_grip":
        rod.visual(
            Cylinder(radius=PLAIN_GRIP_R * scale, length=PLAIN_GRIP_LEN),
            origin=Origin(
                xyz=(0.0, side * (end_y + 0.0375), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=team_mat,
            name="handle",
        )
    elif r.grip_form == "contoured_waisted_grip":
        rod.visual(
            _ergonomic_grip_mesh("handle_mesh", scale, assets=assets),
            origin=Origin(
                xyz=(0.0, side * (end_y + 0.0375), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=team_mat,
            name="handle",
        )
    else:  # ball_knob_grip
        # The knob mesh extrudes along -Y (XZ-workplane normal), so it must be
        # rotated to point OUTBOARD (away from the pitch) on each team side —
        # +Y team needs a pi flip, -Y team needs none. Pointing it inboard (the
        # old, reversed rpy) let the knob dip into the near side panel at full
        # inward slide.
        rod.visual(
            _knob_grip_mesh("handle_mesh", scale, assets=assets),
            origin=Origin(
                xyz=(0.0, side * end_y, 0.0),
                rpy=(math.pi if side > 0 else 0.0, 0.0, 0.0),
            ),
            material=team_mat,
            name="handle",
        )


# ============================================================ build
def build_foosball_table(
    config: FoosballConfig | None = None,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    r = resolve_config(config)
    model = ArticulatedObject(name=r.name, assets=assets)
    pal = PALETTES[r.palette_style]
    mats = {
        key: model.material(f"foosball_{key}_{r.palette_style}", rgba=rgba)
        for key, rgba in pal.items()
    }
    team_mat = {"red": mats["team_a"], "blue": mats["team_b"]}

    wall_zc = (WALL_TOP + PITCH_TOP) / 2.0
    wall_h = WALL_TOP - PITCH_TOP
    full_height_panels = r.leg_form == "side_panels_full_height"
    END_X = r.cab_l / 2.0 - END_WALL_INSET  # end wall a touch inside the cabinet edge
    END_IN_X = END_X - WALL_T / 2.0

    cab = model.part("cabinet")

    # ---- base panel (black) + green pitch floor ----
    cab.visual(
        Box((r.cab_l, CAB_W, BASE_TOP - BASE_BOT)),
        origin=Origin(xyz=(0.0, 0.0, (BASE_TOP + BASE_BOT) / 2.0)),
        material=mats["cabinet"],
        name="cabinet_base",
    )
    cab.visual(
        Box((2.0 * END_X + WALL_T, 0.66, PITCH_TOP - BASE_TOP)),
        origin=Origin(xyz=(0.0, 0.0, (PITCH_TOP + BASE_TOP) / 2.0)),
        material=mats["pitch"],
        name="pitch",
    )

    # ---- side panels (full-height when leg_form folds into them) ----
    if full_height_panels:
        panel_zc = WALL_TOP / 2.0
        y_offsets = (WALL_T - CAB_W / 2.0, CAB_W / 2.0)
        for i in range(2):
            cab.visual(
                _side_panel_mesh(f"side_panel_mesh_{i}", r, full_height=True, assets=assets),
                origin=Origin(xyz=(0.0, y_offsets[i], panel_zc)),
                material=mats["cabinet"],
                name=f"side_panel_{i}",
            )
    else:
        panel_zc = (WALL_TOP + BASE_TOP) / 2.0
        y_offsets = (-(CAB_W / 2.0 - WALL_T), CAB_W / 2.0)
        for i in range(2):
            cab.visual(
                _side_panel_mesh(f"side_panel_mesh_{i}", r, full_height=False, assets=assets),
                origin=Origin(xyz=(0.0, y_offsets[i], panel_zc)),
                material=mats["cabinet"],
                name=f"side_panel_{i}",
            )

    # ---- end walls with goal slots ----
    post_w = INNER_Y - GOAL_HALF_W
    lintel_h = WALL_TOP - GOAL_TOP
    for e, end in ((-1.0, "0"), (1.0, "1")):
        for s, seg in ((-1.0, "a"), (1.0, "b")):
            cab.visual(
                Box((WALL_T, post_w, wall_h)),
                origin=Origin(xyz=(e * END_X, s * (GOAL_HALF_W + post_w / 2.0), wall_zc)),
                material=mats["cabinet"],
                name=f"end_wall_{end}_post_{seg}",
            )
        cab.visual(
            Box((WALL_T, 2.0 * GOAL_HALF_W, lintel_h)),
            origin=Origin(xyz=(e * END_X, 0.0, (WALL_TOP + GOAL_TOP) / 2.0)),
            material=mats["cabinet"],
            name=f"end_wall_{end}_lintel",
        )

    # ---- gray top rim ----
    rim_bot = WALL_TOP - 0.002
    rim_t = RIM_TOP - rim_bot
    for s, idx in ((-1.0, "0"), (1.0, "1")):
        cab.visual(
            Box((r.cab_l - 0.10, 0.05, rim_t)),
            origin=Origin(xyz=(0.0, s * SIDE_Y, rim_bot + rim_t / 2.0)),
            material=mats["rim"],
            name=f"side_rim_{idx}",
        )
    for e, idx in ((-1.0, "0"), (1.0, "1")):
        cab.visual(
            Box((0.05, 2.0 * INNER_Y, rim_t)),
            origin=Origin(xyz=(e * END_X, 0.0, rim_bot + rim_t / 2.0)),
            material=mats["rim"],
            name=f"end_rim_{idx}",
        )

    # ---- pitch markings ----
    cab.visual(
        Box((0.012, 2.0 * INNER_Y, MARK_T)),
        origin=Origin(xyz=(0.0, 0.0, MARK_ZC)),
        material=mats["mark"],
        name="halfway_line",
    )
    ring = cq.Workplane("XY").circle(0.105).circle(0.092).extrude(MARK_T)
    cab.visual(
        mesh_from_cadquery(ring, "center_circle_mesh", assets=assets),
        origin=Origin(xyz=(0.0, 0.0, PITCH_TOP)),
        material=mats["mark"],
        name="center_circle",
    )
    cab.visual(
        Cylinder(radius=0.018, length=MARK_T),
        origin=Origin(xyz=(0.0, 0.0, MARK_ZC)),
        material=mats["mark"],
        name="center_spot",
    )
    box_depth = 0.16
    box_half_w = 0.18
    for e, end in ((-1.0, "0"), (1.0, "1")):
        cab.visual(
            Box((0.012, 2.0 * box_half_w, MARK_T)),
            origin=Origin(xyz=(e * (END_IN_X - box_depth), 0.0, MARK_ZC)),
            material=mats["mark"],
            name=f"goal_box_{end}_front",
        )
        for s, seg in ((-1.0, "a"), (1.0, "b")):
            cab.visual(
                Box((box_depth, 0.012, MARK_T)),
                origin=Origin(xyz=(e * (END_IN_X - box_depth / 2.0), s * box_half_w, MARK_ZC)),
                material=mats["mark"],
                name=f"goal_box_{end}_side_{seg}",
            )

    # ---- score counters above each end ----
    for e, end, tmat in ((-1.0, "0", mats["team_a"]), (1.0, "1", mats["team_b"])):
        for s, seg in ((-1.0, "a"), (1.0, "b")):
            cab.visual(
                Box((0.024, 0.024, 0.05)),
                origin=Origin(xyz=(e * END_X, s * 0.20, RIM_TOP + 0.025)),
                material=mats["cabinet"],
                name=f"score_post_{end}_{seg}",
            )
        cab.visual(
            Cylinder(radius=0.0055, length=0.44),
            origin=Origin(xyz=(e * END_X, 0.0, 0.952), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["steel"],
            name=f"score_rail_{end}",
        )
        for b in range(10):
            cab.visual(
                Cylinder(radius=0.0125, length=0.019),
                origin=Origin(
                    xyz=(e * END_X, -0.185 + b * 0.0205, 0.952),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=tmat,
                name=f"score_bead_{end}_{b}",
            )

    # ---- leg form (Slot B) ----
    if r.leg_form == "splayed_legs_4":
        _emit_splayed_legs(cab, r, mats)
    elif r.leg_form == "tubular_legs_4":
        _emit_tubular_legs(cab, r, mats, assets=assets)
    elif r.leg_form == "cross_x_trestles":
        _emit_cross_trestles(cab, r, mats)
    # side_panels_full_height: no separate leg visuals (panels reach the floor)

    # ---- player rods (Slot A multiplicity): prismatic slide + continuous spin ----
    for rod_spec in r.rods:
        idx = rod_spec.idx
        x = rod_spec.x
        team = rod_spec.team
        n_fig = rod_spec.n_fig
        spacing = rod_spec.spacing
        travel = rod_spec.travel
        prot = rod_protrusion(travel)
        rod_len = CAB_W + 2.0 * prot
        side = -1.0 if team == "red" else 1.0
        end_y = CAB_W / 2.0 + prot

        carrier = model.part(f"rod_{idx}_carrier")
        carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

        rod = model.part(f"rod_{idx}")
        rod.visual(
            Cylinder(radius=ROD_R, length=rod_len),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mats["steel"],
            name="shaft",
        )
        # Slot C grip on the team side
        _emit_handle(rod, r, mats, side=side, end_y=end_y, team_mat=team_mat[team], assets=assets)
        # safety end cap opposite the handle
        rod.visual(
            Cylinder(radius=0.0115, length=0.018),
            origin=Origin(
                xyz=(0.0, -side * (end_y + 0.004), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=mats["cabinet"],
            name="end_cap",
        )
        # Slot D figures (uniformly scaled by fig_scale so adjacent spinning
        # rods cannot clash; offsets along Z scale with the geometry).
        fs = r.fig_scale
        torso = tuple(d * fs for d in FIG_TORSO)
        leg = tuple(d * fs for d in FIG_LEG)
        foot = tuple(d * fs for d in FIG_FOOT)
        for j in range(n_fig):
            yj = (j - (n_fig - 1) / 2.0) * spacing
            p = f"player_{j + 1}"
            rod.visual(
                Box(torso),
                origin=Origin(xyz=(0.0, yj, FIG_TORSO_Z * fs)),
                material=team_mat[team],
                name=f"{p}_torso",
            )
            rod.visual(
                Sphere(FIG_HEAD_R * fs),
                origin=Origin(xyz=(0.0, yj, FIG_HEAD_Z * fs)),
                material=team_mat[team],
                name=f"{p}_head",
            )
            rod.visual(
                Box(leg),
                origin=Origin(xyz=(0.0, yj, FIG_LEG_Z * fs)),
                material=team_mat[team],
                name=f"{p}_legs",
            )
            rod.visual(
                Box(foot),
                origin=Origin(
                    xyz=(FIG_FOOT_X * fs, yj, FIG_FOOT_Z * fs),
                    rpy=(0.0, FIG_FOOT_TILT, 0.0),
                ),
                material=team_mat[team],
                name=f"{p}_foot",
            )

        model.articulation(
            f"rod_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=cab,
            child=carrier,
            origin=Origin(xyz=(x, 0.0, ROD_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-travel, upper=travel),
        )
        model.articulation(
            f"rod_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=carrier,
            child=rod,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    model.meta["slot_choices"] = slot_choices_for_config(r)
    return model


def build_seeded_foosball_table(
    seed: int,
    *,
    assets: AssetContext | None = None,
) -> ArticulatedObject:
    return build_foosball_table(config_from_seed(seed), assets=assets)


# ============================================================ tests
def run_foosball_table_tests(
    object_model: ArticulatedObject,
    config: FoosballConfig,
) -> TestReport:
    r = resolve_config(config)
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")

    # ---- figure geometry clears the pitch and neighbor rods ----
    swing_r = r.swing_r
    ctx.check(
        "figure swing radius clears the pitch",
        swing_r < (ROD_Z - PITCH_TOP) - 0.002,
        details=f"swing radius {swing_r:.4f} m vs rod height above pitch {ROD_Z - PITCH_TOP:.4f} m",
    )
    ctx.check(
        "adjacent spinning rods cannot clash",
        2.0 * swing_r < r.rod_spacing,
        details=f"2*swing={2.0 * swing_r:.4f} vs rod spacing {r.rod_spacing:.4f}",
    )

    # ---- outer rods' spin envelope clears the end walls in X (root fix) ----
    # A figure spins in the X-Z plane, so its max X reach is |x_rod| + swing_r.
    # Keeping that short of the end-wall post near face means an end player can
    # never sweep into a wall post/lintel at ANY slide (Y) + spin combination.
    outer = max(r.rods, key=lambda rs: abs(rs.x))
    ctx.check(
        "outer rod spin envelope clears the end walls",
        abs(outer.x) + swing_r + END_CLEAR <= r.end_in_x + 1e-9,
        details=(
            f"|x|+swing={abs(outer.x) + swing_r:.4f} + clear {END_CLEAR} "
            f"vs end-wall near face {r.end_in_x:.4f}"
        ),
    )

    # ---- per-rod: prismatic slide + continuous spin + figures + handle ----
    for rod_spec in r.rods:
        idx = rod_spec.idx
        team = rod_spec.team
        n_fig = rod_spec.n_fig
        travel = rod_spec.travel
        rod = object_model.get_part(f"rod_{idx}")
        slide = object_model.get_articulation(f"rod_{idx}_slide")
        spin = object_model.get_articulation(f"rod_{idx}_spin")

        ctx.check(
            f"rod_{idx}_slide is prismatic with +/-{travel:.3f} m travel",
            str(slide.articulation_type).lower().endswith("prismatic")
            and slide.motion_limits is not None
            and abs(slide.motion_limits.lower + travel) < 1e-9
            and abs(slide.motion_limits.upper - travel) < 1e-9,
            details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
        )
        ctx.check(
            f"rod_{idx}_spin is continuous",
            str(spin.articulation_type).lower().endswith("continuous"),
            details=f"type={spin.articulation_type}",
        )

        torsos = [v for v in rod.visuals if v.name and v.name.endswith("_torso")]
        ctx.check(
            f"rod_{idx} carries {n_fig} player figures",
            len(torsos) == n_fig,
            details=f"found {len(torsos)} torsos",
        )

        # rod shaft runs through bearing bores in both side panels
        for panel in ("side_panel_0", "side_panel_1"):
            ctx.allow_overlap(
                rod,
                cab,
                elem_a="shaft",
                elem_b=panel,
                reason=(
                    "The steel rod intentionally runs through a bearing bore in the "
                    "side panel; the slight interference represents the wall bushing "
                    "that supports the rod while it slides and spins."
                ),
            )
            ctx.expect_contact(
                rod,
                cab,
                elem_a="shaft",
                elem_b=panel,
                name=f"rod_{idx} shaft is seated in {panel} bearing bore",
            )

        # figure feet hang above the pitch decals at rest pose
        ctx.expect_gap(
            rod,
            cab,
            axis="z",
            positive_elem="player_1_foot",
            negative_elem="pitch",
            min_gap=0.003,
            name=f"rod_{idx} figure feet hang above the pitch",
        )

        # handle sits on the team side, fully outside the cabinet wall
        h_aabb = ctx.part_element_world_aabb(rod, elem="handle")
        side = -1.0 if team == "red" else 1.0
        ok_handle = h_aabb is not None and (
            (side < 0 and h_aabb[1][1] < -CAB_W / 2.0) or (side > 0 and h_aabb[0][1] > CAB_W / 2.0)
        )
        ctx.check(
            f"rod_{idx} handle is outside the {'-Y' if side < 0 else '+Y'} wall ({team} team)",
            ok_handle,
            details=f"handle aabb={h_aabb}",
        )

        # slide limits keep every figure inside the interior walls
        for q in (-travel, travel):
            with ctx.pose({slide: q}):
                for elem in ("player_1_torso", f"player_{n_fig}_torso"):
                    aabb = ctx.part_element_world_aabb(rod, elem=elem)
                    ok = (
                        aabb is not None
                        and aabb[0][1] >= -INNER_Y + 0.002
                        and aabb[1][1] <= INNER_Y - 0.002
                    )
                    ctx.check(
                        f"rod_{idx} {elem} stays inside walls at slide q={q:+.3f}",
                        ok,
                        details=f"aabb={aabb}, interior |y|<={INNER_Y}",
                    )

    # retained insertion: a long-travel rod still spans both wall bores at both
    # slide extremes. Pick the rod with the largest travel.
    long_rod = max(r.rods, key=lambda rs: rs.travel)
    rodL = object_model.get_part(f"rod_{long_rod.idx}")
    slideL = object_model.get_articulation(f"rod_{long_rod.idx}_slide")
    for q in (-long_rod.travel, long_rod.travel):
        with ctx.pose({slideL: q}):
            for panel in ("side_panel_0", "side_panel_1"):
                ctx.expect_overlap(
                    rodL,
                    cab,
                    axes="y",
                    elem_a="shaft",
                    elem_b=panel,
                    min_overlap=WALL_T * 0.8,
                    name=f"rod_{long_rod.idx} shaft stays inserted through {panel} at q={q:+.3f}",
                )

    # outer rods: at both slide extremes and every spin angle, no figure element
    # crosses the end-wall near face in X (the systematic end-player-into-post
    # 穿模). This exercises the slide+spin envelope the motion-QC gate samples.
    outer_idxs = sorted({r.rods[0].idx, r.rods[-1].idx})
    for idx in outer_idxs:
        rod = object_model.get_part(f"rod_{idx}")
        rod_spec = next(rs for rs in r.rods if rs.idx == idx)
        slide = object_model.get_articulation(f"rod_{idx}_slide")
        spin = object_model.get_articulation(f"rod_{idx}_spin")
        elems = [
            f"player_{k}_{part}"
            for k in range(1, rod_spec.n_fig + 1)
            for part in ("torso", "head", "legs", "foot")
        ]
        worst = 0.0
        for q in (-rod_spec.travel, 0.0, rod_spec.travel):
            for step in range(12):
                ang = -math.pi + step * (2.0 * math.pi / 12.0)
                with ctx.pose({slide: q, spin: ang}):
                    for elem in elems:
                        aabb = ctx.part_element_world_aabb(rod, elem=elem)
                        if aabb is not None:
                            worst = max(worst, abs(aabb[0][0]), abs(aabb[1][0]))
        ctx.check(
            f"rod_{idx} figures never reach the end wall across slide+spin",
            worst < r.end_in_x - 1e-4,
            details=f"max |x| reached {worst:.4f} vs end-wall near face {r.end_in_x:.4f}",
        )

    # decisive spin poses on a few rods: feet never touch the pitch
    probe_idxs = sorted(
        {rs.idx for rs in r.rods[:1]}
        | {rs.idx for rs in r.rods[-1:]}
        | {r.rods[len(r.rods) // 2].idx}
    )
    for idx in probe_idxs:
        rod = object_model.get_part(f"rod_{idx}")
        spin = object_model.get_articulation(f"rod_{idx}_spin")
        for ang in (math.pi / 4.0, math.pi / 2.0, math.pi, -math.pi / 4.0):
            with ctx.pose({spin: ang}):
                aabb = ctx.part_element_world_aabb(rod, elem="player_1_foot")
                ctx.check(
                    f"rod_{idx} foot clears pitch at spin {ang:+.2f} rad",
                    aabb is not None and aabb[0][2] > PITCH_TOP + 0.001,
                    details=f"foot aabb={aabb}, pitch top={PITCH_TOP}",
                )

    # ---- goal openings in both end walls ----
    for end in ("0", "1"):
        a = ctx.part_element_world_aabb(cab, elem=f"end_wall_{end}_post_a")
        b = ctx.part_element_world_aabb(cab, elem=f"end_wall_{end}_post_b")
        lintel = ctx.part_element_world_aabb(cab, elem=f"end_wall_{end}_lintel")
        ok = (
            a is not None
            and b is not None
            and lintel is not None
            and (b[0][1] - a[1][1]) > 0.18
            and (lintel[0][2] - PITCH_TOP) > 0.09
        )
        ctx.check(
            f"end wall {end} has an open goal slot",
            ok,
            details=f"post_a={a}, post_b={b}, lintel={lintel}",
        )

    # ---- support reaches the ground ----
    if r.leg_form in ("splayed_legs_4", "tubular_legs_4"):
        for i in range(4):
            aabb = ctx.part_element_world_aabb(cab, elem=f"leg_{i}")
            ctx.check(
                f"leg_{i} touches the ground",
                aabb is not None and -0.006 <= aabb[0][2] <= 0.010,
                details=f"leg aabb={aabb}",
            )
    elif r.leg_form == "cross_x_trestles":
        for idx in (0, 1):
            for letter in ("a", "b"):
                aabb = ctx.part_element_world_aabb(cab, elem=f"trestle_{idx}_bar_{letter}")
                ctx.check(
                    f"trestle_{idx}_bar_{letter} reaches the ground",
                    aabb is not None and aabb[0][2] <= 0.010,
                    details=f"bar aabb={aabb}",
                )
    else:  # side_panels_full_height
        for i in range(2):
            aabb = ctx.part_element_world_aabb(cab, elem=f"side_panel_{i}")
            ctx.check(
                f"side_panel_{i} drops to the floor",
                aabb is not None and aabb[0][2] <= 0.010,
                details=f"panel aabb={aabb}",
            )

    # ---- score counters above each end ----
    for end in ("0", "1"):
        rail = ctx.part_element_world_aabb(cab, elem=f"score_rail_{end}")
        bead = ctx.part_element_world_aabb(cab, elem=f"score_bead_{end}_0")
        ctx.check(
            f"score counter rail {end} sits above the end wall",
            rail is not None and bead is not None and rail[0][2] > RIM_TOP,
            details=f"rail={rail}, bead={bead}",
        )

    # ---- slot_choices recorded ----
    ctx.check(
        "slot_choices recorded",
        tuple(object_model.meta.get("slot_choices", ())) == slot_choices_for_config(r),
        details=str(object_model.meta.get("slot_choices")),
    )

    return ctx.report()


__all__ = (
    "FoosballConfig",
    "ResolvedConfig",
    "build_foosball_table",
    "build_seeded_foosball_table",
    "config_from_seed",
    "resolve_config",
    "run_foosball_table_tests",
    "slot_choices_for_config",
    "slot_choices_for_seed",
)
