from __future__ import annotations

"""Piezo-electric push-button disposable pocket lighter (BIC-style variant).

Variant (ignition_mechanism = piezo_push_button): replaces the knurled flint
spark wheel and thumb lever with a single electronic piezo push-button on top
that presses straight down (PRISMATIC along -Z) to ignite.  The reservoir body,
chrome hood band, and nozzle are retained from the parent.

Frame conventions (object-intrinsic):
- Z is up; the lighter stands on z = 0.
- Y is the long plan axis (~0.025 m): +Y is the flame/notch end ("front").
- X is the short plan axis (~0.012 m).

Parts:
- body (root): translucent royal-blue stadium-plan fuel reservoir, hollow at
  the open top (visible wall rim), grey valve deck recessed inside, chrome
  flame nozzle + collar rising from the deck, polished chrome hood band
  wrapping the upper ~0.015 m (front flame notch, side vent perforations,
  top plate with stem hole), piezo electrode wire near the nozzle, and
  vent slot decorations visible through the hood vents.
- piezo_button: dark-red domed push-button cap with grey stem passing
  through the hood top plate.  PRISMATIC joint along -Z, 0 to 0.002 m
  travel (pressing ignites the piezo crystal).
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- tessellation (mm-scale object) ----
TOL = 5.0e-5
ANG = 0.25

# ---- key dimensions (meters) ----
BODY_LEN = 0.0250   # plan long axis (Y)
BODY_DEP = 0.0120   # plan short axis (X)
BODY_H = 0.0640     # top rim of the blue reservoir
WALL = 0.0012       # reservoir wall thickness at the open top
CAVITY_DEPTH = 0.0040

DECK_Z0 = 0.0595
DECK_Z1 = 0.0610

HOOD_Z0 = 0.0630    # chrome hood band wraps the upper ~0.015 m
HOOD_Z1 = 0.0780
PLATE_THICKNESS = 0.0010
PLATE_HOLE_R = 0.0020   # hole in top plate for button stem

# ---- button ----
BUTTON_CAP_D = 0.0080   # cap diameter
BUTTON_CAP_H = 0.0030   # cap height
BUTTON_STEM_R = 0.0015  # stem radius (clearance fit in plate hole)
BUTTON_STEM_H = 0.0140  # stem length (retained inside hood at max press)
BUTTON_REST_GAP = 0.0020  # gap between cap base and plate top at rest
BUTTON_TRAVEL = 0.0020    # max press distance

# ---- nozzle ----
NOZZLE_Y = 0.0050
NOZZLE_R = 0.0015
NOZZLE_TOP = 0.0745

# ---- vents (repeated sub-parts) ----
N_VENTS = 3
VENT_Y_START = -0.0080
VENT_SPACING = 0.0020
VENT_Z = 0.0710
VENT_R = 0.0006
HOOD_INNER_SPAN = BODY_DEP + 0.0005  # extends past hood walls for connectivity

# Guide boss on the valve deck (contacts button stem bottom at rest)
GUIDE_BOSS_R = 0.0010
GUIDE_BOSS_TOP = 0.0660  # must match button stem bottom at rest (world z)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _reservoir_shell() -> cq.Workplane:
    """Hollow stadium-plan blue reservoir with visible cavity at the top."""
    outer = cq.Workplane("XY").slot2D(BODY_LEN, BODY_DEP, 90).extrude(BODY_H)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - CAVITY_DEPTH)
        .slot2D(BODY_LEN - 2 * WALL, BODY_DEP - 2 * WALL, 90)
        .extrude(CAVITY_DEPTH + 0.002)
    )
    return outer.cut(cavity)


def _valve_deck() -> cq.Workplane:
    """Grey platform recessed inside the reservoir cavity."""
    return (
        cq.Workplane("XY")
        .workplane(offset=DECK_Z0)
        .slot2D(BODY_LEN - 2 * WALL - 0.0004, BODY_DEP - 2 * WALL - 0.0004, 90)
        .extrude(DECK_Z1 - DECK_Z0)
    )


def _hood_band() -> cq.Workplane:
    """Chrome hood: hollow shell with front notch and top plate."""
    # Outer shell
    band = (
        cq.Workplane("XY")
        .workplane(offset=HOOD_Z0)
        .slot2D(BODY_LEN + 0.0012, BODY_DEP + 0.0012, 90)
        .extrude(HOOD_Z1 - HOOD_Z0)
    )
    # Hollow interior (full height so the top plate can fill it back)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=HOOD_Z0 - 0.0005)
        .slot2D(BODY_LEN - 0.0016, BODY_DEP - 0.0016, 90)
        .extrude(HOOD_Z1 - HOOD_Z0 + 0.0010)
    )
    band = band.cut(cavity)

    # Front notch where the flame exits
    notch = (
        cq.Workplane("XY")
        .box(0.0070, 0.0080, 0.0090)
        .translate((0.0, 0.0120, 0.0745))
    )
    band = band.cut(notch)

    # Top plate closes the hood top, with a hole for the button stem
    plate = (
        cq.Workplane("XY")
        .workplane(offset=HOOD_Z1 - PLATE_THICKNESS)
        .slot2D(BODY_LEN + 0.0012, BODY_DEP + 0.0012, 90)
        .extrude(PLATE_THICKNESS)
    )
    plate_hole = (
        cq.Workplane("XY")
        .workplane(offset=HOOD_Z1 - PLATE_THICKNESS - 0.0005)
        .circle(PLATE_HOLE_R)
        .extrude(PLATE_THICKNESS + 0.001)
    )
    plate = plate.cut(plate_hole)
    band = band.union(plate)

    return band


def _vent_slot_geometry() -> Cylinder:
    """Shared helper: thin dark cylinder spanning the hood interior, visible
    through a vent hole.  Used in the vent_slot_{i} loop."""
    return Cylinder(radius=VENT_R * 0.85, length=HOOD_INNER_SPAN)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="piezo_push_button_pocket_lighter")

    # ---- materials ----
    blue = model.material("royal_blue_plastic", rgba=(0.08, 0.13, 0.82, 0.88))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    steel = model.material("steel", rgba=(0.45, 0.46, 0.48, 1.0))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.27, 0.27, 0.29, 1.0))
    deck_grey = model.material("valve_deck_grey", rgba=(0.55, 0.55, 0.56, 1.0))
    button_red = model.material("button_red_plastic", rgba=(0.72, 0.12, 0.10, 1.0))

    # ================ body (root) ================
    body = model.part("body")

    # ---- fuel reservoir layer ----
    body.visual(
        mesh_from_cadquery(
            _reservoir_shell(), "reservoir_shell",
            tolerance=TOL, angular_tolerance=ANG,
        ),
        material=blue,
        name="reservoir_shell",
    )
    body.visual(
        mesh_from_cadquery(
            _valve_deck(), "valve_deck",
            tolerance=TOL, angular_tolerance=ANG,
        ),
        material=deck_grey,
        name="valve_deck",
    )

    # ---- valve / nozzle layer ----
    body.visual(
        Cylinder(radius=NOZZLE_R, length=NOZZLE_TOP - 0.0600),
        origin=Origin(xyz=(0.0, NOZZLE_Y, 0.5 * (NOZZLE_TOP + 0.0600))),
        material=chrome,
        name="flame_nozzle",
    )
    body.visual(
        Cylinder(radius=0.0026, length=0.0045),
        origin=Origin(xyz=(0.0, NOZZLE_Y, 0.0600 + 0.00225)),
        material=chrome,
        name="nozzle_collar",
    )
    # Piezo electrode wire near the nozzle (extends down to valve deck for support)
    electrode_len = 0.0710 - DECK_Z1  # from deck top up into the hood
    body.visual(
        Cylinder(radius=0.0004, length=electrode_len),
        origin=Origin(xyz=(0.0030, NOZZLE_Y - 0.0020, DECK_Z1 + electrode_len / 2.0)),
        material=steel,
        name="piezo_electrode",
    )

    # Guide boss on the valve deck (stem bearing for the push button)
    guide_h = GUIDE_BOSS_TOP - DECK_Z1
    body.visual(
        Cylinder(radius=GUIDE_BOSS_R, length=guide_h),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z1 + guide_h / 2.0)),
        material=deck_grey,
        name="button_guide",
    )

    # ---- hood / wind guard layer ----
    body.visual(
        mesh_from_cadquery(
            _hood_band(), "hood_band",
            tolerance=TOL, angular_tolerance=ANG,
        ),
        material=chrome,
        name="hood_band",
    )

    # ---- vent slot decorations (repeated, named via loop) ----
    for i in range(N_VENTS):
        y = VENT_Y_START + i * VENT_SPACING
        body.visual(
            _vent_slot_geometry(),
            origin=Origin(xyz=(0.0, y, VENT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_grey,
            name=f"vent_slot_{i}",
        )

    # ================ piezo push-button ================
    button = model.part("piezo_button")

    # Domed button cap (KnobGeometry for a realistic dome profile)
    button.visual(
        mesh_from_geometry(
            KnobGeometry(
                BUTTON_CAP_D,
                BUTTON_CAP_H,
                body_style="domed",
                edge_radius=0.0003,
                center=False,
            ),
            "button_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_REST_GAP)),
        material=button_red,
        name="button_cap",
    )

    # Stem passes through the hood top-plate hole into the hollow hood
    stem_center_z = BUTTON_REST_GAP - BUTTON_STEM_H / 2.0
    button.visual(
        Cylinder(radius=BUTTON_STEM_R, length=BUTTON_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, stem_center_z)),
        material=dark_grey,
        name="button_stem",
    )

    # PRISMATIC joint: positive q presses the button downward along -Z
    model.articulation(
        "body_to_piezo_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.0, 0.0, HOOD_Z1)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.5,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    button = object_model.get_part("piezo_button")
    button_joint = object_model.get_articulation("body_to_piezo_button")

    # ---- intentional stem/guide-boss sleeve fit ----
    # When pressed, the stem slides over the guide boss (sleeve bearing).
    ctx.allow_overlap(
        button, body,
        elem_a="button_stem", elem_b="button_guide",
        reason="button stem slides over the guide post during press (sleeve bearing fit)",
    )

    # Proof: stem contacts guide at rest (physical support path)
    ctx.expect_contact(
        button, body,
        elem_a="button_stem", elem_b="button_guide",
        name="button stem seated on guide post at rest",
    )

    # ---- joint type and axis ----
    ctx.check(
        "piezo button is PRISMATIC along -Z",
        button_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(button_joint.axis[2] + 1.0) < 1e-6
        and abs(button_joint.axis[0]) < 1e-6
        and abs(button_joint.axis[1]) < 1e-6,
        details=f"type={button_joint.articulation_type}, axis={button_joint.axis}",
    )

    lim = button_joint.motion_limits
    ctx.check(
        "button travel 0 to ~0.002 m (prismatic downward)",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower) < 1e-9
        and 0.001 <= lim.upper <= 0.005,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # ---- button only moving joint ----
    joint_names = [a.name for a in object_model.articulations]
    ctx.check(
        "exactly one non-fixed joint (piezo button)",
        len(joint_names) == 1 and "body_to_piezo_button" in joint_names,
        details=f"joints={joint_names}",
    )

    # ---- button cap protrudes above the hood at rest ----
    cap = ctx.part_element_world_aabb(button, elem="button_cap")
    band = ctx.part_element_world_aabb(body, elem="hood_band")
    ctx.check(
        "button cap protrudes above the chrome hood at rest",
        bool(cap and band and cap[1][2] > band[1][2] + 0.0005),
        details=f"cap_top={cap[1][2] if cap else None}, band_top={band[1][2] if band else None}",
    )

    # ---- button XY containment within hood opening ----
    ctx.expect_within(
        button, body,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="hood_band",
        margin=0.001,
        name="button cap stays within hood opening (XY)",
    )

    # ---- stem inside the hood vertically ----
    stem = ctx.part_element_world_aabb(button, elem="button_stem")
    ctx.check(
        "button stem stays inside the hood vertically at rest",
        bool(stem and stem[0][2] > HOOD_Z0 - 0.002),
        details=f"stem_bottom={stem[0][2] if stem else None}",
    )

    # ---- decisive pose: pressing button moves cap downward ----
    rest_top = cap[1][2] if cap else 0.0
    rest_bottom = cap[0][2] if cap else 0.0
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed = ctx.part_element_world_aabb(button, elem="button_cap")
        ctx.check(
            "pressing button lowers cap by ~travel distance",
            bool(
                pressed
                and abs((rest_top - pressed[1][2]) - BUTTON_TRAVEL) < 0.0005
            ),
            details=f"rest_top={rest_top}, pressed_top={pressed[1][2] if pressed else None}",
        )
        ctx.check(
            "pressed cap does not penetrate the hood top plate",
            bool(pressed and pressed[0][2] >= HOOD_Z1 - 0.0005),
            details=f"pressed_bottom={pressed[0][2] if pressed else None}",
        )

    # ---- piezo electrode near the nozzle ----
    electrode = ctx.part_element_world_aabb(body, elem="piezo_electrode")
    nozzle = ctx.part_element_world_aabb(body, elem="flame_nozzle")
    ctx.check(
        "piezo electrode is near the flame nozzle",
        bool(
            electrode and nozzle
            and abs(electrode[1][2] - nozzle[1][2]) < 0.010
            and abs(electrode[0][0] - nozzle[0][0]) < 0.006
        ),
        details=f"electrode={electrode}, nozzle={nozzle}",
    )

    # ---- overall dimensions ----
    shell = ctx.part_element_world_aabb(body, elem="reservoir_shell")
    ctx.check(
        "body plan ~0.025 x 0.012 m stadium",
        bool(
            shell
            and abs((shell[1][0] - shell[0][0]) - 0.0120) < 0.0010
            and abs((shell[1][1] - shell[0][1]) - 0.0250) < 0.0010
        ),
        details=f"shell={shell}",
    )

    deck = ctx.part_element_world_aabb(body, elem="valve_deck")
    ctx.check(
        "reservoir reads hollow: valve deck recessed below the open rim",
        bool(deck and shell and deck[1][2] <= shell[1][2] - 0.002),
        details=f"deck={deck}, shell={shell}",
    )

    ctx.check(
        "overall height ~0.08 m (button cap is highest point)",
        bool(cap and 0.078 <= cap[1][2] <= 0.086),
        details=f"cap_top={cap[1][2] if cap else None}",
    )

    # ---- vent slots present and regularly spaced ----
    vent_aabbs = [
        ctx.part_element_world_aabb(body, elem=f"vent_slot_{i}")
        for i in range(N_VENTS)
    ]
    ctx.check(
        f"{N_VENTS} vent slot visuals present on body",
        all(v is not None for v in vent_aabbs),
    )
    if all(v is not None for v in vent_aabbs):
        y_centers = [0.5 * (v[0][1] + v[1][1]) for v in vent_aabbs]
        spacings = [
            y_centers[j + 1] - y_centers[j]
            for j in range(len(y_centers) - 1)
        ]
        ctx.check(
            "vent slots are regularly spaced along Y",
            all(abs(s - VENT_SPACING) < 0.0005 for s in spacings),
            details=f"spacings={spacings}",
        )

    return ctx.report()


object_model = build_object_model()
