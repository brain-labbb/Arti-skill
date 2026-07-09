from __future__ import annotations

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
    mesh_from_geometry,
    sweep_profile_along_spline,
)

# ---------------------------------------------------------------------------
# Twin escalator (two moving staircases side by side), modeled from
# picture/Stairs/Escalator/001.png.
#
# Real mechanism: each unit is an inclined steel truss carrying two side
# balustrades capped by curved rubber handrails. The user-facing moving part
# of each unit is the continuous chain of steps (the "step band") that travels
# up the incline on a PRISMATIC joint whose axis follows the incline
# direction. The handrails are welded to the balustrade tops (a rigidly
# translating closed handrail loop detaches from the newel ends, so it is not
# articulated). The trusses, skirts, balustrade panels, newel ends, landing
# comb plates, handrails and the shared center divider are the fixed root
# structure. Both landings are completely open: each handrail terminates in a
# downward curl at its own balustrade end and nothing spans across the
# walkway.
#
# Geometry layout (world frame):
#   +X = horizontal run direction (toward the upper landing)
#   +Z = vertical rise
#   +Y = transverse (escalator width); the two units sit at +-UNIT_OFFSET
# The incline rises at angle THETA; the step-band travel axis is the unit
# vector (cos T, 0, sin T).
# ---------------------------------------------------------------------------

# Real-world dimensions (meters).
RUN = 6.40  # horizontal length of the inclined section
RISE = 3.40  # vertical floor-to-floor rise of the inclined section
WIDTH = 1.36  # overall escalator width (outer truss to outer truss)
STEP_WIDTH = 0.98  # usable tread width
LANDING_LEN = 1.10  # flat landing length at each end
STEP_SKIRT_GAP = 0.012  # lateral clearance between moving tread edge and skirt

THETA = math.atan2(RISE, RUN)  # incline angle (~28 deg, standard escalator)
INCLINE_LEN = math.hypot(RUN, RISE)  # length along the slope
COS_T = math.cos(THETA)
SIN_T = math.sin(THETA)
AXIS_INCLINE = (COS_T, 0.0, SIN_T)  # up-slope unit vector

# Step band parameters.
STEP_RISE = 0.21  # vertical rise per step (standard ~0.205-0.22 m)
STEP_GO = STEP_RISE / math.tan(THETA)  # horizontal advance per step along run
TREAD_DEPTH = 0.40  # depth of one step tread (front-to-back)
TREAD_THICK = 0.04
RISER_THICK = 0.03
TRAVEL = math.hypot(STEP_GO, STEP_RISE)  # one-step travel along the incline axis

# Step roller / truss guide-track parameters. Each step carries a transverse
# axle with two guide wheels that ride a pair of fixed tracks INSIDE the step
# width; each wheel rests on its track ledge, grounding the moving band to the
# fixed frame. Keeping the wheels inboard of the tread edge avoids the skirts.
ROLLER_R = 0.05  # guide-wheel radius
ROLLER_W = 0.045  # guide-wheel width
ROLLER_DROP = 0.13  # wheel center drop below the tread top
ROLLER_Y = STEP_WIDTH / 2.0 - 0.14  # wheel/track centerline, inboard of tread edge
TRACK_TOP = -(ROLLER_DROP + ROLLER_R)  # track ledge top relative to the tread top
TRACK_SEAT = 0.004  # depth the wheel is captured into the track groove

# Balustrade / handrail lateral placement (outboard of the step band).
BAL_Y = 0.61  # balustrade centerline; outer edge embeds into the truss slab
HANDRAIL_Y = 0.55  # handrail centerline, inboard of the truss so it stays clear
BAL_BASE = 0.05  # balustrade base height above the step line at landings
BAL_H = 0.95  # balustrade panel height
RAIL_Z = BAL_BASE + BAL_H  # handrail centerline height above the step line

# Twin-unit layout: two identical escalators side by side (per the photo) with
# a shared center divider slab between them.
UNIT_GAP = 0.16  # clear gap between the two units' outer truss faces
UNIT_OFFSET = (WIDTH + UNIT_GAP) / 2.0  # unit centerline offset from y = 0
UNITS = (("a", UNIT_OFFSET), ("b", -UNIT_OFFSET))


def _truss_outline() -> list[tuple[float, float]]:
    """Closed XZ outline of the escalator side truss/skirt: incline plus flat
    landing aprons at both ends, drawn as one continuous polygon."""
    deck = 0.55  # truss depth below the step line
    return [
        (-LANDING_LEN, 0.0),
        (0.0, 0.0),
        (RUN, RISE),
        (RUN + LANDING_LEN, RISE),
        (RUN + LANDING_LEN, RISE - deck),
        (RUN, RISE - deck),
        (0.0, -deck),
        (-LANDING_LEN, -deck),
    ]


def _build_truss_side(y_center: float) -> cq.Workplane:
    """One side truss/skirt slab, a 0.06 m thick panel centered at y_center."""
    return (
        cq.Workplane("XZ")
        .polyline(_truss_outline())
        .close()
        .extrude(0.03, both=True)
        .translate((0.0, y_center, 0.0))
    )


def _build_cross_members() -> cq.Workplane:
    """Transverse truss cross-bracing tying the two side trusses together so
    the fixed frame reads as one connected structure. Bars sit well below the
    step band so they never clip the moving steps."""
    asm = None
    n = 6
    # Bars hang a fixed depth below the step line (deck) but stay above the truss
    # bottom edge (0.55) so every bar overlaps both side slabs and the frame
    # reads as one connected body. The depth and bar height keep them well below
    # the lowest step risers (~0.42 m under the step line) at every station.
    deck = 0.47
    bar_h = 0.07
    inner = (WIDTH / 2.0) - 0.03  # reach into both side slabs to fuse the frame
    # Span the inclined run only (skip the bottom-landing extreme where the step
    # return path dips toward the truss floor).
    for i in range(n):
        t = i / (n - 1)
        x = 0.20 + (RUN - 0.30) * t
        z = (x / RUN) * RISE - deck
        bar = (
            cq.Workplane("XY")
            .box(0.08, 2 * inner + 0.10, bar_h)
            .translate((x, 0.0, z))
        )
        asm = bar if asm is None else asm.union(bar)
    return asm


def _build_balustrade(y_center: float) -> cq.Workplane:
    """Glass/metal balustrade panel following the incline above the skirt, with
    rounded newel ends. The lower lip dips below the step line so the panel
    embeds into the truss side slab and reads as one mounted structure."""
    half = 0.012
    x0 = -LANDING_LEN
    x1 = RUN + LANDING_LEN

    def zbase(x: float) -> float:
        # Dip the base below the deck line so it overlaps the truss slab.
        if x <= 0.0:
            return -0.12
        if x >= RUN:
            return RISE - 0.12
        return (x / RUN) * RISE - 0.12

    n = 24
    lower = []
    upper = []
    for i in range(n + 1):
        x = x0 + (x1 - x0) * i / n
        zl = zbase(x)
        lower.append((x, zl))
        upper.append((x, zl + BAL_H + BAL_BASE + 0.12))
    pts = lower + list(reversed(upper))
    # Panel is a thin slab centered on y_center; thickness chosen so its outer
    # edge embeds into the truss side slab.
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(half + 0.01, both=True)
        .translate((0.0, y_center, 0.0))
        .edges("|Y")
        .fillet(0.04)
    )


def _handrail_path_xz() -> list[tuple[float, float]]:
    """XZ centerline of one handrail along the top of the balustrade, with a
    downward return curl at each newel end (each side terminates on its own;
    the rails are never tied across the walkway, so both landings stay open)."""
    x0 = -LANDING_LEN
    x1 = RUN + LANDING_LEN

    def ztop(x: float) -> float:
        if x <= 0.0:
            return RAIL_Z
        if x >= RUN:
            return RISE + RAIL_Z
        return (x / RUN) * RISE + RAIL_Z

    n = 30
    main = [(x0 + (x1 - x0) * i / n, ztop(x0 + (x1 - x0) * i / n)) for i in range(n + 1)]
    # Gentle (<~55 deg) end curls so the sweep frame stays stable with a +Z
    # up-hint while the rail dives toward the balustrade newel end.
    curl_bottom = [
        (x0 - 0.22, RAIL_Z - 0.26),
        (x0 - 0.10, RAIL_Z - 0.10),
    ]
    curl_top = [
        (x1 + 0.10, RISE + RAIL_Z - 0.10),
        (x1 + 0.22, RISE + RAIL_Z - 0.26),
    ]
    return curl_bottom + main + curl_top


def _build_handrail_pair():
    """Both curved rubber handrails of one unit. Each rail runs along its own
    balustrade top and curls down at its own newel ends; there is NO transverse
    member between the two rails, so the walkway entry/exit is fully open.

    The handrail is a downward-open C-channel that grips the balustrade. It is
    built with the SDK spline-sweep helper so the section follows the true 3D
    incline centerline (CadQuery's sweep frame does not lift flat segments to
    the path height reliably)."""
    path_xz = _handrail_path_xz()
    # Downward-open C profile (in the swept frame's local XY = transverse,
    # vertical). Local x = transverse, local y = vertical.
    profile = [
        (-0.05, 0.03),
        (0.05, 0.03),
        (0.05, -0.04),
        (0.035, -0.04),
        (0.035, 0.01),
        (-0.035, 0.01),
        (-0.035, -0.04),
        (-0.05, -0.04),
    ]
    band = None
    for sign in (-1.0, 1.0):
        pts3d = [(x, sign * HANDRAIL_Y, z) for (x, z) in path_xz]
        rail = sweep_profile_along_spline(
            pts3d,
            profile=profile,
            samples_per_segment=4,
            up_hint=(0.0, 0.0, 1.0),
        )
        band = rail if band is None else band.merge(rail)
    return band


def _build_step_band() -> cq.Workplane:
    """The chain of moving steps: each step is a horizontal tread plus a
    vertical riser, carried on roller axles that ride the truss guide tracks.
    The band extends below the lower landing so it stays engaged under the comb
    plate at full travel, but tops out at the upper landing so it does not poke
    above the deck."""
    assembly = None
    # Visible steps run from the bottom landing edge up to the top landing.
    n_steps = int(round(INCLINE_LEN / TRAVEL))
    start_x = -1.0 * STEP_GO  # one hidden step below the bottom comb plate
    start_z = -1.0 * STEP_RISE
    axle_x_off = TREAD_DEPTH * 0.30  # roller axle sits under the tread leading edge
    for i in range(n_steps + 1):
        cx = start_x + i * STEP_GO
        cz = start_z + i * STEP_RISE
        tread = (
            cq.Workplane("XY")
            .box(TREAD_DEPTH, STEP_WIDTH, TREAD_THICK)
            .translate((cx, 0.0, cz))
        )
        riser = (
            cq.Workplane("XY")
            .box(RISER_THICK, STEP_WIDTH, STEP_RISE + TREAD_THICK)
            .translate(
                (cx - TREAD_DEPTH / 2.0 + RISER_THICK / 2.0, 0.0, cz - STEP_RISE / 2.0)
            )
        )
        block = tread.union(riser)
        # Bracket dropping from the tread down to the roller axle line so the
        # step body and its rollers are one connected piece.
        bracket = (
            cq.Workplane("XY")
            .box(0.05, STEP_WIDTH, ROLLER_DROP + 0.02)
            .translate((cx + axle_x_off, 0.0, cz - ROLLER_DROP / 2.0 + 0.01))
        )
        block = block.union(bracket)
        # Transverse roller axle under the tread leading edge.
        axle = (
            cq.Workplane("XY")
            .cylinder(2.0 * ROLLER_Y + ROLLER_W, ROLLER_R * 0.40)
            .rotate((0, 0, 0), (1, 0, 0), 90.0)
            .translate((cx + axle_x_off, 0.0, cz - ROLLER_DROP))
        )
        block = block.union(axle)
        # Two guide-wheel rollers riding a pair of fixed inboard truss tracks.
        for sign in (-1.0, 1.0):
            roller = (
                cq.Workplane("XY")
                .cylinder(ROLLER_W, ROLLER_R)
                .rotate((0, 0, 0), (1, 0, 0), 90.0)
                .translate((cx + axle_x_off, sign * ROLLER_Y, cz - ROLLER_DROP))
            )
            block = block.union(roller)
        assembly = block if assembly is None else assembly.union(block)

    # Two continuous drive-chain bars linking every step axle into one rigid
    # band (the real escalator step chain). They run through the roller-center
    # line just inboard of the wheels so the whole band reads as one piece.
    chain_y = ROLLER_Y - (ROLLER_W / 2.0 + 0.02)
    x_first = start_x + axle_x_off
    x_last = start_x + n_steps * STEP_GO + axle_x_off
    z_first = start_z - ROLLER_DROP
    z_last = start_z + n_steps * STEP_RISE - ROLLER_DROP
    chain_pts = [
        (x_first - 0.03, z_first - 0.025),
        (x_last + 0.03, z_last - 0.025),
        (x_last + 0.03, z_last + 0.025),
        (x_first - 0.03, z_first + 0.025),
    ]
    for sign in (-1.0, 1.0):
        chain = (
            cq.Workplane("XZ")
            .polyline(chain_pts)
            .close()
            .extrude(0.018, both=True)
            .translate((0.0, sign * chain_y, 0.0))
        )
        assembly = assembly.union(chain)
    return assembly


def _roller_bottom_line() -> tuple[tuple[float, float], tuple[float, float]]:
    """Endpoints (x, z) of the at-rest roller-bottom line along the incline.

    The rollers sit at x = cx + axle_x_off, z = cz - ROLLER_DROP, so their
    bottoms trace a straight incline-parallel line. The fixed track top is built
    to coincide with this line so the wheels rest on the track."""
    n_steps = int(round(INCLINE_LEN / TRAVEL))
    start_x = -1.0 * STEP_GO
    start_z = -1.0 * STEP_RISE
    axle_x_off = TREAD_DEPTH * 0.30
    x_first = start_x + axle_x_off
    x_last = start_x + n_steps * STEP_GO + axle_x_off
    z_first = (start_z - ROLLER_DROP) - ROLLER_R
    z_last = (start_z + n_steps * STEP_RISE - ROLLER_DROP) - ROLLER_R
    return (x_first, z_first), (x_last, z_last)


def _build_step_tracks() -> cq.Workplane:
    """Pair of fixed guide-rail tracks (part of the truss) that the step rollers
    ride on. Each rail's top edge coincides with the at-rest roller-bottom line
    so the wheels rest on it, and its lower web reaches down to the cross-bracing
    so the track is tied into the fixed frame."""
    (x0, z0), (x1, z1) = _roller_bottom_line()
    # Seat the track top a few mm into the roller so each wheel is captured by
    # the track groove (a small intentional overlap, allowed in run_tests).
    seat = TRACK_SEAT
    z0 += seat
    z1 += seat
    # Extend slightly past the first/last roller so travel keeps contact.
    x0 -= 0.20
    z0 -= 0.20 * (RISE / RUN)
    x1 += 0.20
    z1 += 0.20 * (RISE / RUN)
    drop = 0.34  # web depth down toward the cross-bracing
    pts = [
        (x0, z0),
        (x1, z1),
        (x1, z1 - drop),
        (x0, z0 - drop),
    ]
    rail = None
    for sign in (-1.0, 1.0):
        web = (
            cq.Workplane("XZ")
            .polyline(pts)
            .close()
            .extrude(0.020, both=True)
            .translate((0.0, sign * ROLLER_Y, 0.0))
        )
        rail = web if rail is None else rail.union(web)
    return rail


def _build_one_comb_plate(x_center: float, z_line: float) -> cq.Workplane:
    """One landing comb/floor plate. The thin floor deck sits clearly above the
    step line so the moving treads pass underneath, and two side rails drop down
    onto the side truss slabs so the plate is tied into the fixed frame."""
    plate_len = LANDING_LEN - 0.10
    deck_thick = 0.025
    deck_bottom = 0.075  # underside well above the tread tops (z ~= +0.02)
    # Central floor deck spanning the full width, kept above the treads.
    deck = (
        cq.Workplane("XY")
        .box(plate_len, WIDTH - 0.06, deck_thick)
        .translate((x_center, 0.0, z_line + deck_bottom + deck_thick / 2.0))
    )
    # Two side rails that bridge from the floor deck down onto the side slabs so
    # the plate connects to the truss without sitting on the moving treads. They
    # straddle the step width (Y outboard of +-STEP_WIDTH/2) so they never touch
    # the moving band.
    rail = None
    rail_z_bottom = z_line - 0.06  # reach below the step line into the side slab
    rail_z_top = z_line + deck_bottom + deck_thick
    rail_h = rail_z_top - rail_z_bottom
    for sign in (-1.0, 1.0):
        leg = (
            cq.Workplane("XY")
            .box(plate_len, 0.10, rail_h)
            .translate(
                (
                    x_center,
                    sign * (WIDTH / 2.0 - 0.07),
                    rail_z_bottom + rail_h / 2.0,
                )
            )
        )
        rail = leg if rail is None else rail.union(leg)
    return deck.union(rail)


def _build_comb_plates() -> cq.Workplane:
    """Landing comb/floor plates at both ends, each tied to the side trusses."""
    plate_len = LANDING_LEN - 0.10
    plate_top = _build_one_comb_plate(RUN + 0.10 + plate_len / 2.0, RISE)
    plate_bot = _build_one_comb_plate(-0.10 - plate_len / 2.0, 0.0)
    return plate_top.union(plate_bot)


def _build_skirt(sign: float) -> cq.Workplane:
    """Smooth inner skirt wall plus side deck shelf flanking the step band on
    one side. The shelf spans outward from the step edge into the truss side
    slab so the inner deck and outer truss read as one connected frame."""
    face_thick = 0.02
    # Inner skirt face surface sits a clearance gap outboard of the tread edge so
    # the moving step band never rubs the fixed skirt.
    y_face_inner = STEP_WIDTH / 2.0 + STEP_SKIRT_GAP
    y_face_center = y_face_inner + face_thick / 2.0
    y_inner = y_face_inner + face_thick  # shelf starts at the outer skirt-face wall
    y_outer = WIDTH / 2.0 - 0.02  # reach into the truss side slab
    span = max(0.04, y_outer - y_inner)
    y_mid = sign * (y_inner + span / 2.0)
    # Inclined deck shelf following the step line, just below the treads.
    shelf = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.05, 0.02),
                (RUN + 0.05, RISE + 0.02),
                (RUN + 0.05, RISE - 0.10),
                (-0.05, -0.10),
            ]
        )
        .close()
        .extrude(span / 2.0, both=True)
        .translate((0.0, y_mid, 0.0))
    )
    # Thin vertical skirt face just outboard of the inner step edge.
    face = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.05, 0.06),
                (RUN + 0.05, RISE + 0.06),
                (RUN + 0.05, RISE - 0.10),
                (-0.05, -0.10),
            ]
        )
        .close()
        .extrude(face_thick / 2.0, both=True)
        .translate((0.0, sign * y_face_center, 0.0))
    )
    return shelf.union(face)


def _build_center_divider() -> cq.Workplane:
    """Shared center divider slab between the two units. It reuses the truss
    side outline and is wide enough to overlap both units' inner truss slabs
    so the twin frame reads as one connected fixed structure."""
    return (
        cq.Workplane("XZ")
        .polyline(_truss_outline())
        .close()
        .extrude(UNIT_GAP / 2.0 + 0.02, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="escalator")

    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    glass = model.material("balustrade_glass", rgba=(0.62, 0.70, 0.78, 0.45))
    rubber = model.material("handrail_rubber", rgba=(0.10, 0.10, 0.12, 1.0))
    tread_metal = model.material("tread_metal", rgba=(0.42, 0.44, 0.47, 1.0))
    floor_plate = model.material("comb_plate", rgba=(0.68, 0.66, 0.40, 1.0))

    half_y = WIDTH / 2.0

    # Build each mesh ONCE; both units reuse the same geometry through visual
    # origins offset along +-Y (keeps compile time near the single-unit cost).
    fixed_geos = [
        (
            mesh_from_cadquery(_build_truss_side(-(half_y - 0.03)), "truss_side_left"),
            dark_steel,
            "truss_side_left",
        ),
        (
            mesh_from_cadquery(_build_truss_side(half_y - 0.03), "truss_side_right"),
            dark_steel,
            "truss_side_right",
        ),
        (
            mesh_from_cadquery(_build_cross_members(), "truss_cross_bracing"),
            dark_steel,
            "truss_cross_bracing",
        ),
        (mesh_from_cadquery(_build_balustrade(BAL_Y), "balustrade_left"), glass, "balustrade_left"),
        (
            mesh_from_cadquery(_build_balustrade(-BAL_Y), "balustrade_right"),
            glass,
            "balustrade_right",
        ),
        (mesh_from_cadquery(_build_comb_plates(), "landing_plates"), floor_plate, "landing_plates"),
        (mesh_from_cadquery(_build_skirt(1.0), "skirt_left"), steel, "skirt_left"),
        (mesh_from_cadquery(_build_skirt(-1.0), "skirt_right"), steel, "skirt_right"),
        (
            mesh_from_cadquery(_build_step_tracks(), "step_guide_tracks"),
            dark_steel,
            "step_guide_tracks",
        ),
        # Handrails are welded to the balustrade tops (fixed structure): a
        # rigidly translating closed handrail loop would detach from the newel
        # ends, so the loop carries no articulation.
        (mesh_from_geometry(_build_handrail_pair(), "handrail_band"), rubber, "handrail_band"),
    ]
    step_band_geo = mesh_from_cadquery(_build_step_band(), "step_band_chain")

    # --- Fixed structure: both units' trusses, cross-bracing, skirts,
    # balustrades, plates, handrails, plus the shared center divider.
    truss = model.part("truss_frame")
    for suffix, y_off in UNITS:
        for geo, mat, base in fixed_geos:
            truss.visual(
                geo,
                origin=Origin(xyz=(0.0, y_off, 0.0)),
                material=mat,
                name=f"{base}_{suffix}",
            )
    truss.visual(
        mesh_from_cadquery(_build_center_divider(), "center_divider"),
        material=dark_steel,
        name="center_divider",
    )

    # --- Moving step bands (primary mechanism), one per unit, each on its own
    # prismatic joint riding up the incline by one step pitch of travel.
    for suffix, y_off in UNITS:
        step_band = model.part(f"step_band_{suffix}")
        step_band.visual(
            step_band_geo,
            origin=Origin(xyz=(0.0, y_off, 0.0)),
            material=tread_metal,
            name=f"step_band_chain_{suffix}",
        )
        model.articulation(
            f"step_travel_{suffix}",
            ArticulationType.PRISMATIC,
            parent=truss,
            child=step_band,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=AXIS_INCLINE,
            motion_limits=MotionLimits(
                effort=8000.0, velocity=0.65, lower=0.0, upper=TRAVEL
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    truss = object_model.get_part("truss_frame")

    # --- Only the two per-unit step joints exist; the handrail loop is welded
    # to the balustrade (a rigidly translating loop would detach at the ends).
    joint_names = sorted(a.name for a in object_model.articulations)
    ctx.check(
        "only the two per-unit step_travel joints exist (no handrail joint)",
        joint_names == ["step_travel_a", "step_travel_b"],
        details=f"joints={joint_names}",
    )

    truss_aabb = ctx.part_world_aabb(truss)
    if truss_aabb is not None:
        run_len = truss_aabb[1][0] - truss_aabb[0][0]
        ctx.check(
            "truss spans the full inclined run plus landings",
            run_len > RUN,
            details=f"run length={run_len:.3f}",
        )
        width = truss_aabb[1][1] - truss_aabb[0][1]
        ctx.check(
            "twin unit spans roughly two escalator widths plus the divider",
            width > 2.0 * WIDTH,
            details=f"overall width={width:.3f}, single width={WIDTH}",
        )

    band_aabbs = {}
    for suffix, y_off in UNITS:
        step_band = object_model.get_part(f"step_band_{suffix}")
        step_travel = object_model.get_articulation(f"step_travel_{suffix}")
        chain = f"step_band_chain_{suffix}"
        tracks = f"step_guide_tracks_{suffix}"

        # The step rollers are captured a few mm into the fixed guide tracks;
        # this is the real step-chain/track engagement grounding the band.
        ctx.allow_overlap(
            step_band,
            truss,
            elem_a=chain,
            elem_b=tracks,
            reason="Step guide wheels are intentionally seated in the fixed truss track groove.",
        )
        ctx.expect_contact(
            step_band,
            truss,
            elem_a=chain,
            elem_b=tracks,
            contact_tol=1e-4,
            name=f"unit {suffix}: step rollers contact the fixed guide track",
        )

        # --- Mechanism type/axis claims.
        ctx.check(
            f"unit {suffix}: step_travel is prismatic",
            str(step_travel.joint_type).endswith("PRISMATIC"),
            details=f"joint_type={step_travel.joint_type}",
        )
        ax = step_travel.axis
        ctx.check(
            f"unit {suffix}: step axis follows the incline (rises along +X and +Z)",
            ax[0] > 0.3 and ax[2] > 0.3 and abs(ax[1]) < 1e-6,
            details=f"axis={ax}",
        )

        # --- Hero parts present.
        ctx.check(
            f"unit {suffix}: balustrades and handrail band present",
            any(v.name == f"balustrade_left_{suffix}" for v in truss.visuals)
            and any(v.name == f"balustrade_right_{suffix}" for v in truss.visuals)
            and any(v.name == f"handrail_band_{suffix}" for v in truss.visuals),
            details="missing balustrade/handrail visuals",
        )
        ctx.check(
            f"unit {suffix}: step band chain present",
            any(v.name == chain for v in step_band.visuals),
            details="missing step band",
        )

        # --- Scale / silhouette checks.
        band_aabb = ctx.part_world_aabb(step_band)
        band_aabbs[suffix] = band_aabb
        if band_aabb is not None:
            height = band_aabb[1][2] - band_aabb[0][2]
            ctx.check(
                f"unit {suffix}: step band spans a realistic floor-to-floor rise",
                height > RISE * 0.6,
                details=f"band height={height:.3f}",
            )
            ctx.check(
                f"unit {suffix}: step band tops out near the upper landing",
                band_aabb[1][2] < RISE + 0.15,
                details=f"band top={band_aabb[1][2]:.3f}, rise={RISE}",
            )

        # --- Handrails ride above the steps and straddle the step width.
        hr_aabb = ctx.part_element_world_aabb(truss, elem=f"handrail_band_{suffix}")
        if band_aabb is not None and hr_aabb is not None:
            ctx.check(
                f"unit {suffix}: handrails ride above the steps on the balustrade",
                hr_aabb[1][2] > band_aabb[1][2],
                details=f"handrail top={hr_aabb[1][2]:.3f}, band top={band_aabb[1][2]:.3f}",
            )
            ctx.check(
                f"unit {suffix}: handrails sit outboard of the step band width",
                hr_aabb[1][1] > band_aabb[1][1] and hr_aabb[0][1] < band_aabb[0][1],
                details=f"handrail y=({hr_aabb[0][1]:.2f},{hr_aabb[1][1]:.2f}), "
                f"band y=({band_aabb[0][1]:.2f},{band_aabb[1][1]:.2f})",
            )

        # --- Steps actually travel UP the incline when the joint is driven,
        # and the welded handrail stays put while they move.
        rest = ctx.part_world_position(step_band)
        with ctx.pose({step_travel: TRAVEL}):
            moved = ctx.part_world_position(step_band)
            hr_posed = ctx.part_element_world_aabb(truss, elem=f"handrail_band_{suffix}")
        if rest is not None and moved is not None:
            ctx.check(
                f"unit {suffix}: driving step_travel moves the band up the slope",
                moved[0] > rest[0] + 1e-4 and moved[2] > rest[2] + 1e-4,
                details=f"rest={rest}, moved={moved}",
            )
        if hr_aabb is not None and hr_posed is not None:
            still = all(
                abs(hr_posed[i][j] - hr_aabb[i][j]) < 1e-9
                for i in (0, 1)
                for j in (0, 1, 2)
            )
            ctx.check(
                f"unit {suffix}: handrail loop stays welded while the steps run",
                still,
                details=f"rest={hr_aabb}, posed={hr_posed}",
            )

    # --- Twin layout: the two step bands are disjoint, mirrored about y = 0.
    aabb_a = band_aabbs.get("a")
    aabb_b = band_aabbs.get("b")
    if aabb_a is not None and aabb_b is not None:
        ctx.check(
            "two side-by-side step bands with a clear gap between them",
            aabb_a[0][1] > aabb_b[1][1] + 0.05,
            details=f"unit a y=({aabb_a[0][1]:.2f},{aabb_a[1][1]:.2f}), "
            f"unit b y=({aabb_b[0][1]:.2f},{aabb_b[1][1]:.2f})",
        )

    return ctx.report()


object_model = build_object_model()
