from __future__ import annotations

# A modern striker-fired semi-automatic pistol (P320-style).
#
# World frame: +X = muzzle/forward, +Y = left side, +Z = up.
# The closed pose rests with the magazine baseplate lowest corner at z ~= 0.
#
# Two-tone: matte black steel slide (angled front/rear cocking serrations,
# optic-ready milled pocket with low-profile red-dot sight, ejection port,
# hollow bore at the muzzle) over an olive-drab
# polymer frame (stippled black grip panels, open-loop undercut trigger
# guard, Picatinny rail with cross slots under the dust cover).
#
# Articulations (all parented to the frame):
#   - slide:          PRISMATIC, axis (-1,0,0), 0..0.045 m rearward travel
#   - trigger:        REVOLUTE about a left-right pin at its top, 0..~25 deg
#   - magazine:       PRISMATIC along the raked grip axis, 0..0.10 m drop
#   - takedown_lever: REVOLUTE about a left-right axis, 0..90 deg

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Layout constants (meters, world frame)
# ---------------------------------------------------------------------------

RAKE = 0.32  # grip rake, radians rearward of vertical
SIN_R = math.sin(RAKE)
COS_R = math.cos(RAKE)

FRAME_HW = 0.014  # frame half-width (Y)
SLIDE_HW = 0.015  # slide half-width (Y)

SLIDE_X0, SLIDE_X1 = -0.105, 0.105  # slide length 0.21 m
SLIDE_Z0, SLIDE_Z1 = 0.118, 0.150  # slide bottom/top
SLIDE_H = SLIDE_Z1 - SLIDE_Z0
FRAME_TOP = SLIDE_Z0  # frame rails meet the slide bottom plane

BORE_Z = 0.133  # bore axis height
BORE_R = 0.0055

# Grip butt: edge perpendicular to the grip axis in side view.
BUTT_F = (-0.054, 0.0074)  # butt front corner (x, z)
BUTT_G = (-0.116, 0.0279)  # butt rear corner (x, z)
BUTT_CX = -0.085
BUTT_CZ = 0.01765

# Magazine joint frame sits at the butt center, pitched by RAKE.
MAG_TRAVEL = 0.10

# Trigger pivot (top of the blade, just inside the frame above the guard
# opening whose underside is at z = 0.085).
TRIG_PIVOT = (0.013, 0.0, 0.086)
TRIG_PULL = math.radians(25.0)

# Takedown lever pivot on the left frame face.
TKD_PIVOT = (0.004, FRAME_HW, 0.104)

SLIDE_TRAVEL = 0.045

_SOLID_CACHE: dict[str, cq.Workplane] = {}


# ---------------------------------------------------------------------------
# CadQuery solids
# ---------------------------------------------------------------------------


def _build_frame_solid() -> cq.Workplane:
    # Side profile (XZ), counterclockwise-ish from the front top corner.
    outline = [
        (0.100, FRAME_TOP),  # front top (under slide muzzle end)
        (0.100, 0.098),  # dust cover front bottom
        (0.046, 0.098),  # dust cover rear / guard front transition
        (0.038, 0.056),  # trigger guard front bottom
        (-0.044, 0.052),  # trigger guard rear bottom (undercut)
        BUTT_F,  # grip front bottom
        BUTT_G,  # grip rear bottom (butt edge perpendicular to grip axis)
        (-0.0925, 0.090),  # grip rear top (backstrap, raked)
        (-0.112, 0.103),  # beavertail tip bottom
        (-0.112, 0.111),  # beavertail tip top
        (-0.100, FRAME_TOP),  # rear top
    ]
    frame = cq.Workplane("XZ").polyline(outline).close().extrude(FRAME_HW, both=True)

    # Open trigger-guard loop: cut the opening through the full width.
    hole = [
        (0.035, 0.085),
        (0.029, 0.062),
        (0.004, 0.058),
        (-0.028, 0.064),
        (-0.020, 0.085),
    ]
    guard_hole = cq.Workplane("XZ").polyline(hole).close().extrude(0.02, both=True)
    frame = frame.cut(guard_hole)

    # Hollow magwell along the raked grip axis (open through the butt face).
    well = (
        cq.Workplane("XY")
        .box(0.025, 0.024, 0.10)
        .rotate((0, 0, 0), (0, 1, 0), math.degrees(RAKE))
        .translate((BUTT_CX + SIN_R * 0.03, 0.0, BUTT_CZ + COS_R * 0.03))
    )
    frame = frame.cut(well)
    return frame


def _build_rail_solid() -> cq.Workplane:
    # Picatinny accessory rail under the dust cover with three cross slots.
    rail = cq.Workplane("XY").box(0.052, 0.022, 0.010).translate((0.072, 0.0, 0.094))
    for xs in (0.057, 0.070, 0.083):
        slot = cq.Workplane("XY").box(0.0055, 0.030, 0.0075).translate((xs, 0.0, 0.09275))
        rail = rail.cut(slot)
    return rail


def _build_slide_solid() -> cq.Workplane:
    # Built centered at the origin, then shifted so local z = 0 is the slide
    # bottom plane (slide part frame sits at world (0, 0, SLIDE_Z0) at q=0).
    s = cq.Workplane("XY").box(SLIDE_X1 - SLIDE_X0, 2 * SLIDE_HW, SLIDE_H)
    s = s.edges("|X and >Z").chamfer(0.004)

    # Hollow bore at the muzzle.
    bore = (
        cq.Workplane("XY")
        .cylinder(0.050, BORE_R)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((0.090, 0.0, BORE_Z - (SLIDE_Z0 + SLIDE_H / 2.0)))
    )
    s = s.cut(bore)

    # Ejection port: notch through the right wall and top edge.
    port = cq.Workplane("XY").box(0.046, 0.015, 0.017).translate((0.035, -0.0105, 0.011 - 0.016))
    s = s.cut(port)

    # Optic-ready cut: recessed milled pocket on the top rear deck.
    # In the centered frame the top face is at z = +SLIDE_H/2.
    pocket_depth = 0.008
    pocket_z = SLIDE_H / 2.0 - pocket_depth / 2.0  # center of pocket
    pocket = (
        cq.Workplane("XY")
        .box(0.060, 0.025, pocket_depth)
        .translate((-0.070, 0.0, pocket_z))
    )
    s = s.cut(pocket)

    # Mounting screw holes in the pocket floor (two small blind holes).
    hole_depth = 0.005
    hole_z = SLIDE_H / 2.0 - pocket_depth - hole_depth / 2.0
    for dx in (-0.018, 0.018):
        hole = (
            cq.Workplane("XY")
            .cylinder(hole_depth, 0.0015)
            .translate((-0.070 + dx, 0.0, hole_z))
        )
        s = s.cut(hole)

    return s.translate((0.0, 0.0, SLIDE_H / 2.0))


def _build_trigger_solid() -> cq.Workplane:
    # Trigger-local frame: pivot pin at the origin, blade hangs below and is
    # curved (bulges forward). The root above z=0 hides inside the frame.
    tr = (
        cq.Workplane("XZ")
        .moveTo(-0.0045, 0.002)
        .lineTo(0.0045, 0.002)
        .threePointArc((0.0078, -0.010), (0.0008, -0.0215))
        .lineTo(-0.0030, -0.0190)
        .threePointArc((-0.0026, -0.009), (-0.0045, 0.002))
        .close()
        .extrude(0.004, both=True)
    )
    return tr


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="striker_fired_pistol")

    model.material("slide_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("frame_olive", rgba=(0.42, 0.43, 0.36, 1.0))
    model.material("stipple_black", rgba=(0.09, 0.09, 0.09, 1.0))
    model.material("sight_black", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("serration_gray", rgba=(0.28, 0.28, 0.30, 1.0))
    model.material("barrel_steel", rgba=(0.46, 0.46, 0.48, 1.0))
    model.material("mag_graphite", rgba=(0.20, 0.20, 0.21, 1.0))
    model.material("baseplate_black", rgba=(0.07, 0.07, 0.07, 1.0))
    model.material("optic_lens", rgba=(0.15, 0.22, 0.18, 1.0))

    # ----- frame (root) ----------------------------------------------------
    frame = model.part("frame")
    frame_solid = _build_frame_solid()
    _SOLID_CACHE["frame"] = frame_solid
    frame.visual(mesh_from_cadquery(frame_solid, "frame_body"), material="frame_olive", name="frame_body")
    frame.visual(
        mesh_from_cadquery(_build_rail_solid(), "accessory_rail"),
        material="frame_olive",
        name="accessory_rail",
    )
    # Stippled black grip panels (both sides, raked with the grip).
    for tag, sy in (("left", 1.0), ("right", -1.0)):
        frame.visual(
            Box((0.032, 0.003, 0.052)),
            origin=Origin(xyz=(-0.0766, sy * 0.0155, 0.048), rpy=(0.0, RAKE, 0.0)),
            material="stipple_black",
            name=f"{tag}_grip_panel",
        )
    # Static slide-stop lever detail on the left frame face.
    frame.visual(
        Box((0.024, 0.0025, 0.0045)),
        origin=Origin(xyz=(-0.032, 0.0146, 0.110)),
        material="sight_black",
        name="slide_stop_lever",
    )

    # ----- slide -----------------------------------------------------------
    slide = model.part("slide")
    slide_solid = _build_slide_solid()
    _SOLID_CACHE["slide"] = slide_solid
    slide.visual(mesh_from_cadquery(slide_solid, "slide_body"), material="slide_black", name="slide_body")
    # Barrel block visible through the ejection port.
    slide.visual(
        Cylinder(radius=0.0085, length=0.054),
        origin=Origin(xyz=(0.035, 0.0, BORE_Z - SLIDE_Z0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="barrel_steel",
        name="barrel_block",
    )
    # Sights: front sight post and optic-ready red-dot block on the rear deck.
    slide.visual(
        Box((0.006, 0.0065, 0.004)),
        origin=Origin(xyz=(0.097, 0.0, 0.0335)),
        material="sight_black",
        name="front_sight",
    )
    # Low-profile red-dot optic housing seated in the milled pocket.
    slide.visual(
        Box((0.036, 0.022, 0.010)),
        origin=Origin(xyz=(-0.070, 0.0, 0.030)),
        material="sight_black",
        name="optic_sight_block",
    )
    # Optic lens window (recessed into the top of the housing).
    slide.visual(
        Box((0.018, 0.016, 0.003)),
        origin=Origin(xyz=(-0.072, 0.0, 0.034)),
        material="optic_lens",
        name="optic_lens_window",
    )
    # Mounting screws on the pocket floor (repeated via loop).
    for i in range(2):
        dx = -0.018 + i * 0.036
        slide.visual(
            Cylinder(radius=0.002, length=0.002),
            origin=Origin(xyz=(-0.070 + dx, 0.0, 0.025)),
            material="serration_gray",
            name=f"optic_screw_{i}",
        )
    # Angled cocking serrations: rear and front groups on both flanks.
    rear_xs = [-0.094 + i * 0.0046 for i in range(8)]
    front_xs = [0.062 + i * 0.005 for i in range(6)]
    for group, xs_list in (("rear", rear_xs), ("front", front_xs)):
        for i, sx in enumerate(xs_list):
            for tag, sy in (("left", 1.0), ("right", -1.0)):
                slide.visual(
                    Box((0.0015, 0.0014, 0.020)),
                    origin=Origin(xyz=(sx, sy * 0.0148, 0.015), rpy=(0.0, 0.21, 0.0)),
                    material="serration_gray",
                    name=f"{group}_serration_{i}_{tag}",
                )

    model.articulation(
        "frame_to_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_Z0)),
        axis=(-1.0, 0.0, 0.0),  # positive q retracts the slide rearward
        motion_limits=MotionLimits(effort=80.0, velocity=2.5, lower=0.0, upper=SLIDE_TRAVEL),
    )

    # ----- trigger ----------------------------------------------------------
    trigger = model.part("trigger")
    trigger_solid = _build_trigger_solid()
    _SOLID_CACHE["trigger"] = trigger_solid
    trigger.visual(
        mesh_from_cadquery(trigger_solid, "trigger_blade"),
        material="sight_black",
        name="trigger_blade",
    )

    model.articulation(
        "frame_to_trigger",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=TRIG_PIVOT),
        axis=(0.0, 1.0, 0.0),  # positive q swings the blade rearward (-X)
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=TRIG_PULL),
    )

    # ----- takedown lever ---------------------------------------------------
    lever = model.part("takedown_lever")
    lever.visual(
        Cylinder(radius=0.0075, length=0.006),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="frame_olive",
        name="lever_boss",
    )
    lever.visual(
        Cylinder(radius=0.0028, length=0.002),
        origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="sight_black",
        name="lever_screw",
    )
    lever.visual(
        Box((0.024, 0.003, 0.0075)),
        origin=Origin(xyz=(0.0135, 0.003, 0.0)),
        material="frame_olive",
        name="lever_tab",
    )

    model.articulation(
        "frame_to_takedown_lever",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=TKD_PIVOT),
        axis=(0.0, 1.0, 0.0),  # positive q rotates the forward tab downward
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )

    # ----- magazine ---------------------------------------------------------
    magazine = model.part("magazine")
    magazine.visual(
        Box((0.021, 0.021, 0.076)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material="mag_graphite",
        name="mag_body",
    )
    magazine.visual(
        Box((0.040, 0.030, 0.013)),
        origin=Origin(xyz=(0.0, 0.0, -0.0055)),
        material="baseplate_black",
        name="mag_baseplate",
    )

    model.articulation(
        "frame_to_magazine",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=magazine,
        origin=Origin(xyz=(BUTT_CX, 0.0, BUTT_CZ), rpy=(0.0, RAKE, 0.0)),
        axis=(0.0, 0.0, -1.0),  # positive q drops the mag down-rearward
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=MAG_TRAVEL),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def _probe_volume(solid: cq.Workplane, center: tuple[float, float, float], size: float) -> float:
    probe = cq.Workplane("XY").box(size, size, size).translate(center)
    try:
        inter = solid.intersect(probe)
        return float(sum(s.Volume() for s in inter.solids().vals()))
    except Exception:
        return 0.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    slide = object_model.get_part("slide")
    trigger = object_model.get_part("trigger")
    lever = object_model.get_part("takedown_lever")
    magazine = object_model.get_part("magazine")

    j_slide = object_model.get_articulation("frame_to_slide")
    j_trigger = object_model.get_articulation("frame_to_trigger")
    j_lever = object_model.get_articulation("frame_to_takedown_lever")
    j_mag = object_model.get_articulation("frame_to_magazine")

    # Intentional local embeddings.
    ctx.allow_overlap(
        frame,
        trigger,
        elem_a="frame_body",
        elem_b="trigger_blade",
        reason="trigger root passes up through the frame slot to its hidden pivot pin",
    )
    ctx.allow_overlap(
        frame,
        lever,
        elem_a="frame_body",
        elem_b="lever_boss",
        reason="takedown lever boss seats into its bore in the frame side wall",
    )
    ctx.allow_overlap(
        frame,
        magazine,
        elem_a="frame_body",
        elem_b="mag_baseplate",
        reason="extended magazine baseplate seats flush against the grip heel",
    )

    # --- joint plan: types, axes, ranges ------------------------------------
    sl = j_slide.motion_limits
    ctx.check(
        "slide is prismatic rearward along barrel axis with 0.045 m travel",
        j_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(j_slide.axis[0] + 1.0) < 1e-6
        and abs(j_slide.axis[1]) < 1e-6
        and abs(j_slide.axis[2]) < 1e-6
        and sl is not None
        and abs(sl.upper - 0.045) < 1e-9,
        details=f"axis={j_slide.axis}, limits={sl}",
    )
    tl = j_trigger.motion_limits
    ctx.check(
        "trigger is revolute about a left-right pin with ~25 deg pull",
        j_trigger.articulation_type == ArticulationType.REVOLUTE
        and abs(j_trigger.axis[1] - 1.0) < 1e-6
        and tl is not None
        and abs(tl.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={j_trigger.axis}, limits={tl}",
    )
    ll = j_lever.motion_limits
    ctx.check(
        "takedown lever is revolute about a left-right axis with 90 deg range",
        j_lever.articulation_type == ArticulationType.REVOLUTE
        and abs(j_lever.axis[1] - 1.0) < 1e-6
        and ll is not None
        and abs(ll.upper - math.pi / 2.0) < 1e-6,
        details=f"axis={j_lever.axis}, limits={ll}",
    )
    ml = j_mag.motion_limits
    ctx.check(
        "magazine is prismatic along the raked grip axis with 0.10 m drop",
        j_mag.articulation_type == ArticulationType.PRISMATIC
        and abs(j_mag.axis[2] + 1.0) < 1e-6
        and abs(j_mag.origin.rpy[1] - RAKE) < 1e-9
        and ml is not None
        and abs(ml.upper - 0.10) < 1e-9,
        details=f"axis={j_mag.axis}, rpy={j_mag.origin.rpy}, limits={ml}",
    )

    # --- real-world scale and grounding -------------------------------------
    slide_aabb = ctx.part_world_aabb(slide)
    ctx.check(
        "slide spans ~0.21 m overall length",
        slide_aabb is not None and 0.205 <= slide_aabb[1][0] - slide_aabb[0][0] <= 0.218,
        details=f"slide aabb={slide_aabb}",
    )
    ctx.check(
        "overall height ~0.15 m (slide top + optic sight block)",
        slide_aabb is not None and 0.148 <= slide_aabb[1][2] <= 0.160,
        details=f"slide aabb={slide_aabb}",
    )
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame with grip panels is ~0.034 m wide",
        frame_aabb is not None and 0.030 <= frame_aabb[1][1] - frame_aabb[0][1] <= 0.038,
        details=f"frame aabb={frame_aabb}",
    )
    mag_aabb = ctx.part_world_aabb(magazine)
    ctx.check(
        "magazine baseplate grounds the closed pose at z ~= 0",
        mag_aabb is not None and -0.002 <= mag_aabb[0][2] <= 0.004,
        details=f"magazine aabb={mag_aabb}",
    )

    # --- assembly relationships ----------------------------------------------
    ctx.expect_gap(
        slide,
        frame,
        axis="z",
        max_penetration=0.0005,
        max_gap=0.0015,
        name="slide rides directly on the frame rails",
    )
    ctx.expect_within(
        magazine,
        frame,
        axes="x",
        margin=0.004,
        name="seated magazine stays inside the grip footprint",
    )
    ctx.expect_overlap(
        magazine,
        frame,
        axes="z",
        min_overlap=0.05,
        name="seated magazine is retained deep in the magwell",
    )
    ctx.expect_overlap(
        trigger,
        frame,
        axes="xz",
        min_overlap=0.002,
        name="trigger hangs inside the guard opening",
    )

    # --- hollow features (probe the cached CadQuery solids) ------------------
    ctx.check(
        "bore is hollow at the muzzle",
        _probe_volume(_SOLID_CACHE["slide"], (0.100, 0.0, BORE_Z - SLIDE_Z0), 0.004) < 1e-12,
        details="material found inside the muzzle bore",
    )
    ctx.check(
        "trigger guard is an open loop",
        _probe_volume(_SOLID_CACHE["frame"], (0.005, 0.0, 0.070), 0.004) < 1e-12,
        details="material found inside the guard opening",
    )

    # --- optic-ready cut deck features ----------------------------------------
    ctx.check(
        "optic milled pocket is recessed into the slide top deck",
        _probe_volume(_SOLID_CACHE["slide"], (-0.070, 0.0, 0.028), 0.006) < 1e-12,
        details="material found inside the optic pocket recess",
    )
    ctx.check(
        "optic sight block protrudes above the slide top deck",
        slide_aabb is not None and slide_aabb[1][2] > SLIDE_Z0 + SLIDE_H + 0.001,
        details=f"slide top z={slide_aabb[1][2] if slide_aabb else None}, deck={SLIDE_Z0 + SLIDE_H}",
    )

    # --- decisive articulated poses ------------------------------------------
    rest_slide = ctx.part_world_aabb(slide)
    with ctx.pose({j_slide: SLIDE_TRAVEL}):
        open_slide = ctx.part_world_aabb(slide)
    ctx.check(
        "retracted slide moves 0.045 m rearward",
        rest_slide is not None
        and open_slide is not None
        and abs((rest_slide[0][0] - open_slide[0][0]) - SLIDE_TRAVEL) < 1e-6,
        details=f"rest={rest_slide}, open={open_slide}",
    )

    rest_trig = ctx.part_world_aabb(trigger)
    with ctx.pose({j_trigger: TRIG_PULL}):
        pulled_trig = ctx.part_world_aabb(trigger)
    ctx.check(
        "pulled trigger blade swings rearward (off-axis blade proves rotation)",
        rest_trig is not None
        and pulled_trig is not None
        and pulled_trig[0][0] < rest_trig[0][0] - 0.005,
        details=f"rest={rest_trig}, pulled={pulled_trig}",
    )

    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({j_lever: math.pi / 2.0}):
        rot_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "rotated takedown lever tab points downward (off-axis tab proves rotation)",
        rest_lever is not None
        and rot_lever is not None
        and rot_lever[0][2] < rest_lever[0][2] - 0.010,
        details=f"rest={rest_lever}, rotated={rot_lever}",
    )

    with ctx.pose({j_mag: MAG_TRAVEL}):
        dropped_mag = ctx.part_world_aabb(magazine)
    ctx.check(
        "dropped magazine exits the magwell below the grip",
        dropped_mag is not None and dropped_mag[1][2] < 0.01,
        details=f"dropped={dropped_mag}",
    )
    ctx.check(
        "magazine drops down and rearward along the raked grip axis",
        mag_aabb is not None
        and dropped_mag is not None
        and dropped_mag[0][2] < mag_aabb[0][2] - 0.09
        and dropped_mag[0][0] < mag_aabb[0][0] - 0.02,
        details=f"rest={mag_aabb}, dropped={dropped_mag}",
    )

    return ctx.report()


object_model = build_object_model()
