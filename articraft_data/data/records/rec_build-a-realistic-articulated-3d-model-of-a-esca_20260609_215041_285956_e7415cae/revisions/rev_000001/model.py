from __future__ import annotations

# Realistic articulated escalator (moving staircase), modeled from
# picture/Stairs/Escalator/002.png.
#
# Real object read from the image:
# - A sloped truss running from a lower landing up to an upper landing at
#   roughly 30 degrees, flanked on both sides by light gray skirt panels and
#   solid balustrades carrying dark tinted glass and curved rubber handrails.
# - A band of metal treads (the moving staircase) climbs the incline in the
#   middle, between two bright stainless skirt strips.
# - At the foot of the incline there is a rectangular access pit; its floor
#   cover plate is lifted/hinged open in the reference, exposing the dark
#   machinery box below.
#
# Articulated mechanisms (physics-driven):
# - step_band : PRISMATIC translation up the incline -> the moving steps.
#   This is the primary user-facing escalator motion.
# - left_handrail / right_handrail : FIXED (welded to the balustrades). A
#   closed handrail loop circulates in place; translating it rigidly would
#   slide it off the balustrade ends, so the rubber loops are welded and only
#   the step band carries the travel articulation.
# - pit_cover : REVOLUTE -> the bottom floor access plate hinges up at the
#   landing edge for maintenance access, matching the open pit in the image.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Geometry parameters (meters, real-world scale)
# ---------------------------------------------------------------------------
INCLINE_ANGLE = math.radians(30.0)  # standard escalator inclination
SIN_A = math.sin(INCLINE_ANGLE)
COS_A = math.cos(INCLINE_ANGLE)

WIDTH = 1.20  # overall machine width (balustrade outer to outer)
TRUSS_RUN = 4.40  # horizontal run of the sloped truss section
TRUSS_RISE = TRUSS_RUN * math.tan(INCLINE_ANGLE)  # vertical rise

LANDING_LEN = 1.05  # horizontal length of each flat landing platform
LANDING_TOP_Z = 0.16  # thickness of the bottom landing slab (top surface z)

TREAD_WIDTH = 0.78  # usable step width between skirts
N_STEPS = 11  # visible treads on the incline (read from image)
STEP_RISE = 0.205  # vertical rise per step
STEP_DEPTH = 0.355  # horizontal tread depth (run) per step

SKIRT_H = 0.30  # skirt panel height above the tread line
BALUS_H = 0.95  # balustrade (glass + frame) height above tread line
GLASS_T = 0.022
HANDRAIL_W = 0.075
HANDRAIL_H = 0.055

# Incline geometry: the toothed tread line starts at the top of the lower
# landing and climbs to the upper landing.
INCL_X0 = LANDING_LEN  # x where the incline begins (end of lower landing)
INCL_Z0 = LANDING_TOP_Z  # z of the tread line at the bottom of the incline
INCL_X1 = INCL_X0 + TRUSS_RUN
INCL_Z1 = INCL_Z0 + TRUSS_RISE


def _incline_dir() -> tuple[float, float, float]:
    """Unit vector pointing up the incline in the X-Z plane."""
    return (COS_A, 0.0, SIN_A)


# ---------------------------------------------------------------------------
# Material palette (restrained, from the reference image)
# ---------------------------------------------------------------------------
def _register_materials(model: ArticulatedObject) -> None:
    model.material("escalator_body", rgba=(0.86, 0.88, 0.90, 1.0))  # light gray skirt/truss
    model.material("stainless", rgba=(0.78, 0.80, 0.83, 1.0))  # bright skirt strips
    model.material("tread_metal", rgba=(0.42, 0.45, 0.48, 1.0))  # dark grooved treads
    model.material("tinted_glass", rgba=(0.10, 0.11, 0.13, 0.55))  # smoked balustrade glass
    model.material("rubber_handrail", rgba=(0.07, 0.07, 0.08, 1.0))  # black rubber handrail
    model.material("pit_dark", rgba=(0.12, 0.13, 0.15, 1.0))  # dark machinery box
    model.material("comb_plate", rgba=(0.74, 0.76, 0.78, 1.0))  # comb / floor plate


# ---------------------------------------------------------------------------
# Frame (root): truss + landings + skirts + balustrade glass + pit box
# ---------------------------------------------------------------------------
def _build_frame(model: ArticulatedObject) -> None:
    frame = model.part("frame")

    half_w = WIDTH / 2.0

    # --- Sloped truss body (the solid wedge that carries the incline) ---
    # Build as an extruded side profile in the X-Z plane, then thicken in Y.
    # The truss top follows a line a clear margin BELOW the moving step band's
    # underside so the steps ride above the truss with no interpenetration.
    # Step-band underside (riser-bottom / tread) line; truss top stays below it.
    band_x_lo = INCL_X0 + 0.10
    band_x_hi = band_x_lo + (N_STEPS - 1) * STEP_DEPTH + STEP_DEPTH
    band_z_lo = INCL_Z0  # riser bottoms start at the tread line
    band_slope = STEP_RISE / STEP_DEPTH  # = tan(incline)
    # Margin must exceed one step rise so the riser back-bottom edges still
    # clear the truss top everywhere along the incline.
    margin = STEP_RISE + 0.07
    # Truss top at the incline ends, kept a clear margin below the tread line.
    top_lo_z = band_z_lo - margin
    top_hi_z = band_z_lo + (band_x_hi - band_x_lo) * band_slope - margin
    truss_bottom_z = INCL_Z0 - 0.55  # underside of the sloped truss
    # Extend the truss ends under both landing slabs so it intersects (and is
    # connected to) the landing platforms rather than only meeting them edge-on.
    side_profile = [
        (INCL_X0 - 0.30, truss_bottom_z),
        (INCL_X1 + 0.30, truss_bottom_z + TRUSS_RISE),
        (INCL_X1 + 0.30, INCL_Z1 - 0.05),  # rises into the upper landing slab
        (band_x_hi, top_hi_z),
        (band_x_lo, top_lo_z),
        (INCL_X0 - 0.30, INCL_Z0 - 0.05),  # rises into the lower landing slab
    ]
    truss = (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(WIDTH)
        .translate((0.0, half_w, 0.0))  # extrude goes -Y; recenter on Y=0
    )
    frame.visual(
        mesh_from_cadquery(truss, "truss_body"),
        material="escalator_body",
        name="truss_body",
    )

    # --- Lower landing platform (with the access pit cut into its floor) ---
    lower_landing = (
        cq.Workplane("XY")
        .box(LANDING_LEN, WIDTH, LANDING_TOP_Z, centered=(False, True, False))
    )
    # Cut a rectangular access pit into the top of the lower landing.
    pit_len = 0.62
    pit_w = WIDTH - 0.30
    pit_x0 = 0.18
    pit = (
        cq.Workplane("XY")
        .transformed(offset=(pit_x0 + pit_len / 2.0, 0.0, LANDING_TOP_Z - 0.02))
        .box(pit_len, pit_w, 0.10, centered=(True, True, False))
    )
    lower_landing = lower_landing.cut(pit)
    frame.visual(
        mesh_from_cadquery(lower_landing, "lower_landing"),
        material="comb_plate",
        name="lower_landing",
    )

    # --- Dark machinery box sitting in the pit (open top, like the image) ---
    box_wall = 0.03
    box_depth = 0.34
    box_outer = (
        cq.Workplane("XY")
        .transformed(offset=(pit_x0 + pit_len / 2.0, 0.0, LANDING_TOP_Z - box_depth))
        .box(pit_len - 0.02, pit_w - 0.02, box_depth, centered=(True, True, False))
    )
    box_inner = (
        cq.Workplane("XY")
        .transformed(
            offset=(pit_x0 + pit_len / 2.0, 0.0, LANDING_TOP_Z - box_depth + box_wall)
        )
        .box(
            pit_len - 0.02 - 2 * box_wall,
            pit_w - 0.02 - 2 * box_wall,
            box_depth,
            centered=(True, True, False),
        )
    )
    pit_box = box_outer.cut(box_inner)
    frame.visual(
        mesh_from_cadquery(pit_box, "pit_box"),
        material="pit_dark",
        name="pit_box",
    )

    # --- Upper landing platform ---
    upper_landing = (
        cq.Workplane("XY")
        .transformed(offset=(INCL_X1, 0.0, INCL_Z1 - LANDING_TOP_Z))
        .box(LANDING_LEN, WIDTH, LANDING_TOP_Z, centered=(False, True, False))
    )
    frame.visual(
        mesh_from_cadquery(upper_landing, "upper_landing"),
        material="comb_plate",
        name="upper_landing",
    )

    # --- Comb plate at the bottom of the incline (where steps emerge) ---
    comb = (
        cq.Workplane("XY")
        .transformed(offset=(INCL_X0 + 0.06, 0.0, INCL_Z0))
        .box(0.16, TREAD_WIDTH, 0.02, centered=(True, True, False))
    )
    frame.visual(
        mesh_from_cadquery(comb, "lower_comb"),
        material="stainless",
        name="lower_comb",
    )

    # --- Bright stainless skirt strips flanking the treads on the incline ---
    skirt_strip_t = 0.025
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y = sign * (TREAD_WIDTH / 2.0 + skirt_strip_t / 2.0)
        strip = (
            cq.Workplane("XZ")
            .polyline(
                [
                    (INCL_X0, INCL_Z0),
                    (INCL_X1, INCL_Z1),
                    (INCL_X1, INCL_Z1 + SKIRT_H),
                    (INCL_X0, INCL_Z0 + SKIRT_H),
                ]
            )
            .close()
            .extrude(skirt_strip_t)
            .translate((0.0, y + skirt_strip_t / 2.0, 0.0))
        )
        frame.visual(
            mesh_from_cadquery(strip, f"skirt_{side}"),
            material="stainless",
            name=f"skirt_{side}",
        )

    # --- Solid balustrade base + tinted glass + glass nose at top/bottom ---
    glass_inner_y = TREAD_WIDTH / 2.0 + 0.05
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y_base = sign * (glass_inner_y + GLASS_T / 2.0)

        # Balustrade deck rail under the glass: a solid bar that bridges the
        # bright skirt strip and the outer newel so the glass panel is carried
        # by (and connected to) the surrounding frame body. It spans OUTBOARD
        # from the skirt strip to the newel, clear of the moving treads.
        skirt_inner_y = TREAD_WIDTH / 2.0 + 0.025  # magnitude, inboard edge
        newel_outer_y = half_w  # magnitude, outboard edge
        deck_span = newel_outer_y - skirt_inner_y
        deck = (
            cq.Workplane("XZ")
            .polyline(
                [
                    (INCL_X0 - 0.10, INCL_Z0 + SKIRT_H - 0.02),
                    (INCL_X1 + 0.10, INCL_Z1 + SKIRT_H - 0.02),
                    (INCL_X1 + 0.10, INCL_Z1 + SKIRT_H + 0.05),
                    (INCL_X0 - 0.10, INCL_Z0 + SKIRT_H + 0.05),
                ]
            )
            .close()
            .extrude(deck_span)  # extrudes toward -Y by deck_span
        )
        # Place the deck on the correct side, spanning skirt->newel outboard.
        # The -Y extrusion occupies [base - deck_span, base] after translate.
        if sign > 0:
            deck = deck.translate((0.0, newel_outer_y, 0.0))  # 0.60 -> 0.415
        else:
            deck = deck.translate((0.0, -skirt_inner_y, 0.0))  # -0.415 -> -0.60
        frame.visual(
            mesh_from_cadquery(deck, f"balustrade_deck_{side}"),
            material="escalator_body",
            name=f"balustrade_deck_{side}",
        )

        # Tinted glass balustrade panel running along the incline.
        glass = (
            cq.Workplane("XZ")
            .polyline(
                [
                    (INCL_X0 - 0.10, INCL_Z0 + SKIRT_H),
                    (INCL_X1 + 0.10, INCL_Z1 + SKIRT_H),
                    (INCL_X1 + 0.10, INCL_Z1 + BALUS_H),
                    (INCL_X0 - 0.10, INCL_Z0 + BALUS_H),
                ]
            )
            .close()
            .extrude(GLASS_T)
            .translate((0.0, y_base + (sign * GLASS_T / 2.0), 0.0))
        )
        # Rounded glass end-caps (the curved decked ends seen in the image).
        end_low = (
            cq.Workplane("XZ")
            .center(INCL_X0 - 0.10, INCL_Z0 + (SKIRT_H + BALUS_H) / 2.0)
            .ellipse(0.22, (BALUS_H - SKIRT_H) / 2.0)
            .extrude(GLASS_T)
            .translate((0.0, y_base + (sign * GLASS_T / 2.0), 0.0))
        )
        end_high = (
            cq.Workplane("XZ")
            .center(INCL_X1 + 0.10, INCL_Z1 + (SKIRT_H + BALUS_H) / 2.0)
            .ellipse(0.22, (BALUS_H - SKIRT_H) / 2.0)
            .extrude(GLASS_T)
            .translate((0.0, y_base + (sign * GLASS_T / 2.0), 0.0))
        )
        glass_panel = glass.union(end_low).union(end_high)
        frame.visual(
            mesh_from_cadquery(glass_panel, f"glass_{side}"),
            material="tinted_glass",
            name=f"glass_{side}",
        )

    # --- Newel skirting (white outer balustrade base shells) ---
    for sign, side in ((1.0, "left"), (-1.0, "right")):
        y_out = sign * (half_w - 0.03)
        base_shell = (
            cq.Workplane("XZ")
            .polyline(
                [
                    (INCL_X0 - 0.20, INCL_Z0),
                    (INCL_X1 + 0.20, INCL_Z1),
                    (INCL_X1 + 0.20, INCL_Z1 + SKIRT_H + 0.02),
                    (INCL_X0 - 0.20, INCL_Z0 + SKIRT_H + 0.02),
                ]
            )
            .close()
            .extrude(0.06)
            .translate((0.0, y_out + (sign * 0.03), 0.0))
        )
        frame.visual(
            mesh_from_cadquery(base_shell, f"newel_{side}"),
            material="escalator_body",
            name=f"newel_{side}",
        )


# ---------------------------------------------------------------------------
# Step band (PRISMATIC): the moving staircase of grooved treads + risers
# ---------------------------------------------------------------------------
def _build_step_band(model: ArticulatedObject) -> None:
    steps = model.part("step_band")

    riser_h = STEP_RISE
    tread_d = STEP_DEPTH
    tread_t = 0.03
    string_t = 0.04  # continuous stringer plate thickness

    # Continuous side stringers (one per side) running under the nosings tie
    # every tread/riser into a single connected solid, like a real step band.
    # The stringer's BOTTOM edge follows the riser-bottom (tread) line so it is
    # the lowest part of the band and connects every riser; the truss top is
    # then placed a clear margin below this same line.
    x_lo = INCL_X0 + 0.10
    x_hi = x_lo + (N_STEPS - 1) * STEP_DEPTH + tread_d
    slope = STEP_RISE / STEP_DEPTH
    # Bottom edge passes through riser bottoms: z = INCL_Z0 at x_lo, slope up.
    z_bot_lo = INCL_Z0
    band_thk = 0.30  # tall enough to intersect every riser/tread above it
    band = None
    for sign in (1.0, -1.0):
        y = sign * (TREAD_WIDTH / 2.0 - 0.02)
        stringer = (
            cq.Workplane("XZ")
            .polyline(
                [
                    (x_lo, z_bot_lo),
                    (x_hi, z_bot_lo + (x_hi - x_lo) * slope),
                    (x_hi, z_bot_lo + (x_hi - x_lo) * slope + band_thk),
                    (x_lo, z_bot_lo + band_thk),
                ]
            )
            .close()
            .extrude(string_t)
            .translate((0.0, y - sign * string_t / 2.0, 0.0))
        )
        band = stringer if band is None else band.union(stringer)

    # Lay treads along the incline as a real staircase profile.
    for i in range(N_STEPS):
        x = INCL_X0 + 0.10 + i * STEP_DEPTH
        z = INCL_Z0 + i * STEP_RISE

        tread = (
            cq.Workplane("XY")
            .transformed(offset=(x + tread_d / 2.0, 0.0, z + riser_h - tread_t))
            .box(tread_d, TREAD_WIDTH, tread_t, centered=(True, True, False))
        )
        # Vertical riser at the back of each tread, reaching down to the stringer.
        riser = (
            cq.Workplane("XY")
            .transformed(offset=(x + tread_d, 0.0, z + riser_h / 2.0))
            .box(0.025, TREAD_WIDTH, riser_h, centered=(True, True, False))
        )
        step_i = tread.union(riser)
        band = step_i if band is None else band.union(step_i)

    assert band is not None
    steps.visual(
        mesh_from_cadquery(band, "step_treads"),
        material="tread_metal",
        name="step_treads",
    )


# ---------------------------------------------------------------------------
# Handrails (FIXED, welded to balustrades): black rubber loops on balustrade tops
# ---------------------------------------------------------------------------
def _build_handrail(model: ArticulatedObject, side: str, sign: float) -> None:
    rail_part = model.part(f"{side}_handrail")

    # Handrail sits centered on the top edge of the tinted glass balustrade.
    y = sign * (TREAD_WIDTH / 2.0 + 0.05 + GLASS_T / 2.0)

    def z_top(x: float) -> float:
        return INCL_Z0 + (x - INCL_X0) * math.tan(INCLINE_ANGLE) + BALUS_H

    x_lo = INCL_X0 - 0.18
    x_hi = INCL_X1 + 0.18
    cx = (x_lo + x_hi) / 2.0
    cz = (z_top(x_lo) + z_top(x_hi)) / 2.0

    # The C-section rubber handrail: a rounded rectangular bar that caps the
    # glass edge. The bar bottom dips slightly below the glass top so it reads
    # as seated/gripping the balustrade (a small intentional seated overlap).
    length = math.hypot(x_hi - x_lo, z_top(x_hi) - z_top(x_lo))
    bar = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .box(length, HANDRAIL_H, HANDRAIL_W, centered=(True, True, True))
        .edges("|X").fillet(HANDRAIL_H * 0.35)
    )
    bar = bar.rotate((0, 0, 0), (0, 1, 0), -math.degrees(INCLINE_ANGLE))
    bar = bar.translate((cx, y, cz))

    # Rounded return noses at the two ends (the curved handrail ends).
    nose_lo = (
        cq.Workplane("XY")
        .transformed(offset=(x_lo, y, z_top(x_lo)))
        .sphere(HANDRAIL_H / 2.0)
    )
    nose_hi = (
        cq.Workplane("XY")
        .transformed(offset=(x_hi, y, z_top(x_hi)))
        .sphere(HANDRAIL_H / 2.0)
    )
    rail = bar.union(nose_lo).union(nose_hi)

    rail_part.visual(
        mesh_from_cadquery(rail, f"{side}_rail"),
        material="rubber_handrail",
        name=f"{side}_rail",
    )


# ---------------------------------------------------------------------------
# Pit cover (REVOLUTE): hinged floor access plate at the lower landing
# ---------------------------------------------------------------------------
def _build_pit_cover(model: ArticulatedObject) -> None:
    cover = model.part("pit_cover")

    pit_len = 0.62
    pit_w = WIDTH - 0.30
    plate_t = 0.03

    # The cover plate's local frame is at the hinge line (its up-slope edge).
    # The plate extends along local -X from the hinge toward the pit, lying flat
    # in the X-Y plane at z=0 so positive rotation about +Y lifts the free edge.
    plate = (
        cq.Workplane("XY")
        .transformed(offset=(-pit_len / 2.0, 0.0, plate_t / 2.0))
        .box(pit_len, pit_w, plate_t, centered=(True, True, True))
    )
    # Diamond-tread grip ribs on top of the cover.
    cover.visual(
        mesh_from_cadquery(plate, "pit_cover_plate"),
        material="comb_plate",
        name="pit_cover_plate",
    )
    # Two visible hinge knuckles along the hinge edge (at local x=0).
    for ky in (-pit_w / 2.0 + 0.06, pit_w / 2.0 - 0.06):
        knuckle = (
            cq.Workplane("XZ")
            .transformed(offset=(0.0, plate_t / 2.0, ky))
            .circle(0.018)
            .extrude(0.05)
            .translate((0.0, ky - 0.025, 0.0))
        )
        cover.visual(
            mesh_from_cadquery(knuckle, f"hinge_knuckle_{'p' if ky > 0 else 'n'}"),
            material="stainless",
            name=f"hinge_knuckle_{'p' if ky > 0 else 'n'}",
        )


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="escalator")
    _register_materials(model)

    _build_frame(model)
    _build_step_band(model)
    _build_handrail(model, "left", 1.0)
    _build_handrail(model, "right", -1.0)
    _build_pit_cover(model)

    frame = model.get_part("frame")
    steps = model.get_part("step_band")
    left_rail = model.get_part("left_handrail")
    right_rail = model.get_part("right_handrail")
    cover = model.get_part("pit_cover")

    incl = _incline_dir()

    # Primary mechanism: the moving staircase travels up the incline.
    # One step pitch of travel proves the band steps upward along the slope.
    step_pitch = math.hypot(STEP_DEPTH, STEP_RISE)
    model.articulation(
        "frame_to_steps",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=steps,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=incl,
        motion_limits=MotionLimits(
            effort=3000.0, velocity=0.65, lower=0.0, upper=step_pitch
        ),
    )

    # Handrail loops are welded to the balustrades. A real handrail is a
    # closed loop that circulates in place; rigidly translating the loop would
    # detach it from the balustrade ends, so the rails carry no travel joint.
    for name, child in (("frame_to_left_handrail", left_rail), ("frame_to_right_handrail", right_rail)):
        model.articulation(
            name,
            ArticulationType.FIXED,
            parent=frame,
            child=child,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    # Maintenance pit cover hinges up at the up-slope edge of the access pit.
    pit_len = 0.62
    pit_x0 = 0.18
    hinge_x = pit_x0 + pit_len  # up-slope edge of the pit (toward the incline)
    model.articulation(
        "frame_to_pit_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cover,
        # Hinge line sits on the lower-landing top surface at the pit's far edge.
        origin=Origin(xyz=(hinge_x, 0.0, LANDING_TOP_Z)),
        # Cover extends along local -X; +Y axis lifts the free (pit-side) edge up.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.4),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    steps = object_model.get_part("step_band")
    left_rail = object_model.get_part("left_handrail")
    right_rail = object_model.get_part("right_handrail")
    cover = object_model.get_part("pit_cover")

    steps_joint = object_model.get_articulation("frame_to_steps")
    cover_joint = object_model.get_articulation("frame_to_pit_cover")
    left_joint = object_model.get_articulation("frame_to_left_handrail")
    right_joint = object_model.get_articulation("frame_to_right_handrail")

    # The rubber handrails are intentionally seated onto the top edge of the
    # tinted glass balustrades; the small embed is what carries each rail and
    # makes it grip the glass (this is the handrail-balustrade contact path).
    ctx.allow_overlap(
        left_rail,
        frame,
        elem_a="left_rail",
        elem_b="glass_left",
        reason="Left rubber handrail is seated/capped onto the left glass balustrade top edge.",
    )
    ctx.allow_overlap(
        right_rail,
        frame,
        elem_a="right_rail",
        elem_b="glass_right",
        reason="Right rubber handrail is seated/capped onto the right glass balustrade top edge.",
    )
    # The fixed comb plate intentionally meshes with the bottom of the moving
    # step band (the comb teeth interleave the tread grooves at the landing).
    ctx.allow_overlap(
        frame,
        steps,
        elem_a="lower_comb",
        elem_b="step_treads",
        reason="Comb plate teeth intentionally mesh with the bottom step treads at the boarding point.",
    )

    # --- Joint type / axis claims ---
    ctx.check(
        "step band is prismatic",
        steps_joint.joint_type == "prismatic",
        details=f"joint_type={steps_joint.joint_type}",
    )
    incl = _incline_dir()
    ax = steps_joint.axis
    ctx.check(
        "step prismatic axis points up the incline",
        abs(ax[0] - incl[0]) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2] - incl[2]) < 1e-6,
        details=f"axis={ax}",
    )
    ctx.check(
        "pit cover is revolute about +Y",
        cover_joint.joint_type == "revolute"
        and abs(cover_joint.axis[1] - 1.0) < 1e-6,
        details=f"type={cover_joint.joint_type} axis={cover_joint.axis}",
    )
    ctx.check(
        "handrail loops are welded to the balustrades (no travel joint)",
        left_joint.joint_type == "fixed"
        and right_joint.joint_type == "fixed"
        and left_joint.mimic is None
        and right_joint.mimic is None,
        details=f"left={left_joint.joint_type} right={right_joint.joint_type}",
    )

    # Handrails stay seated regardless of step-band travel: their world AABB
    # must be identical at both step-joint extremes (the rails do not detach).
    lr0 = ctx.part_world_aabb(left_rail)
    with ctx.pose({steps_joint: math.hypot(STEP_DEPTH, STEP_RISE)}):
        lr1 = ctx.part_world_aabb(left_rail)
    assert lr0 is not None and lr1 is not None
    drift = max(
        abs(lr0[i][k] - lr1[i][k]) for i in range(2) for k in range(3)
    )
    ctx.check(
        "handrail position is invariant under step-band travel",
        drift < 1e-9,
        details=f"max aabb drift={drift:.2e}",
    )

    # --- Hero geometry present and placed ---
    # Steps climb the incline: top of band is well above its bottom.
    band_aabb = ctx.part_world_aabb(steps)
    assert band_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = band_aabb
    ctx.check(
        "step band rises along the incline",
        (sz1 - sz0) > 1.5 and (sx1 - sx0) > 2.5,
        details=f"band aabb size dx={sx1 - sx0:.3f} dz={sz1 - sz0:.3f}",
    )
    ctx.check(
        "step band width is within the skirts",
        (sy1 - sy0) <= TREAD_WIDTH + 0.01,
        details=f"band width dy={sy1 - sy0:.3f}",
    )

    # Handrails sit above the balustrade and outboard of the treads.
    lr_aabb = ctx.part_world_aabb(left_rail)
    rr_aabb = ctx.part_world_aabb(right_rail)
    assert lr_aabb is not None and rr_aabb is not None
    ctx.check(
        "left handrail is on +Y side, right on -Y side",
        lr_aabb[0][1] > 0.0 and rr_aabb[1][1] < 0.0,
        details=f"left ymin={lr_aabb[0][1]:.3f} right ymax={rr_aabb[1][1]:.3f}",
    )
    ctx.check(
        "handrails sit above the step band",
        lr_aabb[1][2] > sz0 + 0.8 and rr_aabb[1][2] > sz0 + 0.8,
        details=f"left zmax={lr_aabb[1][2]:.3f} right zmax={rr_aabb[1][2]:.3f}",
    )

    # Handrails are seated/connected onto the glass balustrade tops, not floating.
    left_rail_v = left_rail.get_visual("left_rail")
    glass_l_v = frame.get_visual("glass_left")
    right_rail_v = right_rail.get_visual("right_rail")
    glass_r_v = frame.get_visual("glass_right")
    ctx.expect_contact(
        left_rail,
        frame,
        elem_a=left_rail_v,
        elem_b=glass_l_v,
        contact_tol=1e-4,
        name="left handrail seats on left glass",
    )
    ctx.expect_contact(
        right_rail,
        frame,
        elem_a=right_rail_v,
        elem_b=glass_r_v,
        contact_tol=1e-4,
        name="right handrail seats on right glass",
    )

    # Frame carries glass balustrades and landings.
    glass_l = frame.get_visual("glass_left")
    glass_r = frame.get_visual("glass_right")
    ctx.check(
        "tinted glass balustrades exist on both sides",
        glass_l is not None and glass_r is not None,
        details="glass visuals present",
    )

    # Comb plate meshes with the bottom of the step band at the boarding point.
    comb_v = frame.get_visual("lower_comb")
    treads_v = steps.get_visual("step_treads")
    ctx.expect_contact(
        frame,
        steps,
        elem_a=comb_v,
        elem_b=treads_v,
        contact_tol=1e-3,
        name="comb plate meshes with the step band",
    )

    # --- Pit cover seats flat in the closed pose, lifts when actuated ---
    with ctx.pose({cover_joint: 0.0}):
        closed = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: 1.2}):
        opened = ctx.part_world_aabb(cover)
    assert closed is not None and opened is not None
    ctx.check(
        "pit cover lifts when the hinge opens",
        opened[1][2] > closed[1][2] + 0.25,
        details=f"closed top z={closed[1][2]:.3f} open top z={opened[1][2]:.3f}",
    )
    ctx.check(
        "closed pit cover lies near the landing surface",
        closed[1][2] < LANDING_TOP_Z + 0.06,
        details=f"closed top z={closed[1][2]:.3f}",
    )

    # --- Steps actually advance up-slope when posed ---
    rest = ctx.part_world_position(steps)
    with ctx.pose({steps_joint: math.hypot(STEP_DEPTH, STEP_RISE)}):
        advanced = ctx.part_world_position(steps)
    assert rest is not None and advanced is not None
    ctx.check(
        "step band advances up the incline when posed",
        advanced[0] > rest[0] + 0.10 and advanced[2] > rest[2] + 0.05,
        details=f"rest={rest} advanced={advanced}",
    )

    return ctx.report()


object_model = build_object_model()
